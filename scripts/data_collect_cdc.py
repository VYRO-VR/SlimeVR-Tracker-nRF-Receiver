#!/usr/bin/env python
# /// script
# dependencies = [
#   "pyserial",
# ]
# ///

"""
SlimeVR Raw Sensor Data Collector

Reads raw sensor data from the SlimeVR receiver via CDC ACM serial port
and saves it to a binary file for offline VQF parameter optimization.

Binary framing protocol (from receiver):
  [0xAA][0x55][length][payload...][rssi][rx_ticks(4)][CRC-8]

Packet types in payload:
  0x10: Raw IMU (48 bytes) - float gyro+accel (+optional body-frame aligned mag) + T-Cal temp
  0x11: Raw Mag (17 bytes) - float body-frame aligned magnetometer
  meta: raw TX Hz, optional chip/fusion Hz after mag_id
  gyrQuat: accumulated gyro quaternion + accel/body-frame aligned mag

Output: CSV file with columns depending on mode:
  raw mode:      seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp,discontinuity
  gyr_quat mode: seq,qw,qx,qy,qz,ax,ay,az,mx,my,mz,temp,discontinuity
"""

import argparse
import math
import struct
import sys
import time
from pathlib import Path
from data_collect_reorder import SampleReorder


from data_collect_common import (
    META_MASK_ACCEL,
    META_MASK_BASIC,
    META_MASK_GYRO,
    META_MASK_MAG,
    META_MASK_TCAL_POINTS,
    META_MASK_TCAL_STATE,
    MetadataRepairScheduler,
    make_control_sender,
    receiver_serial_for_cdc,
)
# SlimeNRF Receiver USB IDs
SLIME_VID = 0x1209
SLIME_PID = 0x7690


def crc8_ccitt(data: bytes, init: int = 0x07) -> int:
    """CRC-8 CCITT (polynomial 0x07)."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class SensorMetadata:
    """Stores fields from a raw metadata packet."""

    def __init__(self):
        self.gyro_range = 0.0
        self.accel_range = 0.0
        self.gyro_odr = 0.0  # raw TX Hz (prepare timebase)
        self.accel_odr = 0.0
        self.mag_odr = 0.0
        self.imu_id = 0
        self.mag_id = 0
        self.gyro_chip_odr = None  # optional trailer when >0
        self.gyro_fusion_odr = None  # optional trailer when >0
        self.received = False

    def parse(self, payload: bytes):
        if len(payload) < 24:
            return
        self.gyro_range = struct.unpack_from("<f", payload, 2)[0]
        self.accel_range = struct.unpack_from("<f", payload, 6)[0]
        self.gyro_odr = struct.unpack_from("<f", payload, 10)[0]
        self.accel_odr = struct.unpack_from("<f", payload, 14)[0]
        self.mag_odr = struct.unpack_from("<f", payload, 18)[0]
        self.imu_id = payload[22]
        self.mag_id = payload[23]
        self.gyro_chip_odr = None
        self.gyro_fusion_odr = None
        if len(payload) >= 28:
            chip = struct.unpack_from("<f", payload, 24)[0]
            if math.isfinite(chip) and chip > 0:
                self.gyro_chip_odr = chip
                if len(payload) >= 32:
                    fusion = struct.unpack_from("<f", payload, 28)[0]
                    if math.isfinite(fusion) and fusion > 0:
                        self.gyro_fusion_odr = fusion
        self.received = True

    def __str__(self):
        parts = [
            f"Gyro: {self.gyro_range:.0f} dps send@{self.gyro_odr:.0f} Hz",
        ]
        if self.gyro_chip_odr is not None:
            parts[0] += f" chip@{self.gyro_chip_odr:.0f} Hz"
        if self.gyro_fusion_odr is not None:
            parts[0] += f" fusion@{self.gyro_fusion_odr:.0f} Hz"
        parts.append(
            f"Accel: {self.accel_range:.0f} g @ {self.accel_odr:.0f} Hz, "
            f"Mag: {self.mag_odr:.0f} Hz, "
            f"IMU ID: {self.imu_id}, Mag ID: {self.mag_id}"
        )
        return ", ".join(parts)


class CalibrationData:
    """Stores calibration data from raw cal packets."""

    def __init__(self):
        self.accel_BAinv = None
        self.mag_BAinv = None
        self.gyro_bias = None
        self.gyro_sens_scale = None
        self.tcal_enabled = False
        self.tcal_num_points = 0
        self.tcal_temp_min = 0.0
        self.tcal_temp_max = 0.0
        self.tcal_correction_offset = None
        self.tcal_state_received = False
        self.tcal_points = []
        self.tcal_complete = False
        self._tcal_chunks = {}

    @staticmethod
    def _is_valid_tcal_point(point):
        return all(math.isfinite(v) for v in point) and not all(v == 0.0 for v in point)

    @classmethod
    def _normalize_tcal_points(cls, points):
        deduped = {}
        for point in points:
            if cls._is_valid_tcal_point(point):
                deduped[f"{point[0]:.2f}"] = point
        return [deduped[key] for key in sorted(deduped, key=float)]

    def parse(self, payload: bytes):
        if len(payload) < 4:
            return
        sub_type = payload[2]
        if sub_type == 0x01 and len(payload) >= 51:
            self.accel_BAinv = list(struct.unpack_from("<12f", payload, 3))
        elif sub_type == 0x02 and len(payload) >= 51:
            self.mag_BAinv = list(struct.unpack_from("<12f", payload, 3))
        elif sub_type == 0x03 and len(payload) >= 27:
            self.gyro_bias = list(struct.unpack_from("<3f", payload, 3))
            self.gyro_sens_scale = list(struct.unpack_from("<3f", payload, 15))
        elif sub_type == 0x04 and len(payload) >= 26:
            self.tcal_state_received = True
            self.tcal_enabled = bool(payload[3])
            total_count = struct.unpack_from("<H", payload, 4)[0]
            if total_count != self.tcal_num_points:
                self._tcal_chunks.clear()
            self.tcal_num_points = total_count
            self.tcal_temp_min = struct.unpack_from("<f", payload, 6)[0]
            self.tcal_temp_max = struct.unpack_from("<f", payload, 10)[0]
            self.tcal_correction_offset = list(struct.unpack_from("<3f", payload, 14))
            self._complete_tcal()
        elif sub_type == 0x05 and len(payload) >= 7:
            chunk_idx = payload[3]
            total_count = struct.unpack_from("<H", payload, 4)[0]
            count = payload[6]
            if self.tcal_state_received and total_count != self.tcal_num_points:
                return
            if total_count != self.tcal_num_points:
                self._tcal_chunks.clear()
            if total_count == 0:
                self.tcal_num_points = 0
                self._complete_tcal()
                return
            expected_count = min(2, total_count - 2 * chunk_idx)
            if expected_count <= 0 or count != expected_count or len(payload) < 7 + 16 * count:
                return
            points = [struct.unpack_from("<4f", payload, 7 + j * 16) for j in range(count)]
            if not all(self._is_valid_tcal_point(point) for point in points):
                return
            self.tcal_num_points = total_count
            self._tcal_chunks[chunk_idx] = points
            self._complete_tcal()

    def _complete_tcal(self):
        expected = (self.tcal_num_points + 1) // 2
        self.tcal_points = []
        self.tcal_complete = False
        if self.tcal_state_received and all(i in self._tcal_chunks for i in range(expected)):
            points = self._normalize_tcal_points(
                point for i in range(expected) for point in self._tcal_chunks[i]
            )
            if len(points) == self.tcal_num_points:
                self.tcal_points = points
                self.tcal_complete = True

    def write_to_file(self, f):
        f.write("\n# Calibration data (from tracker retained memory)\n")
        if self.accel_BAinv is not None and len(self.accel_BAinv) == 12:
            f.write(f"acc_cal_bias={','.join(f'{v:.9g}' for v in self.accel_BAinv[:3])}\n")
            f.write(f"acc_cal_matrix={','.join(f'{v:.9g}' for v in self.accel_BAinv[3:])}\n")
        if self.mag_BAinv is not None and len(self.mag_BAinv) == 12:
            f.write(f"mag_cal_bias={','.join(f'{v:.9g}' for v in self.mag_BAinv[:3])}\n")
            f.write(f"mag_cal_matrix={','.join(f'{v:.9g}' for v in self.mag_BAinv[3:])}\n")
        if self.gyro_bias is not None:
            f.write(f"gyro_bias={','.join(f'{v:.9g}' for v in self.gyro_bias)}\n")
        if self.gyro_sens_scale is not None:
            f.write(f"gyro_sens_scale={','.join(f'{v:.9g}' for v in self.gyro_sens_scale)}\n")
        if self.tcal_complete and self.tcal_points:
            entries = [f"{t:.2f}:{bx:.5f},{by:.5f},{bz:.5f}" for t, bx, by, bz in self.tcal_points]
            f.write(f"gyro_tcal={';'.join(entries)}\n")
        f.write(f"tcal_declared_points={self.tcal_num_points}\n")
        f.write(f"tcal_complete={int(self.tcal_complete)}\n")
    @property
    def has_data(self):
        return bool(self.accel_BAinv or self.mag_BAinv or self.gyro_bias or self.tcal_state_received)


def parse_raw_imu(payload: bytes, meta: SensorMetadata):
    """Parse legacy raw IMU packet (float format, no timestamp). Returns a single sample dict."""
    if len(payload) < 42:
        return None

    _tracker_id = payload[1]
    seq = struct.unpack_from(">H", payload, 2)[0]

    # Float gyro/accel (native byte order from ARM, little-endian)
    gx, gy, gz = struct.unpack_from("<fff", payload, 4)
    ax, ay, az = struct.unpack_from("<fff", payload, 16)

    flags = payload[40]
    temp_c = None
    if len(payload) >= 45:
        tcal_temp_c = struct.unpack_from("<f", payload, 41)[0]
        if math.isfinite(tcal_temp_c) and -100.0 < tcal_temp_c < 150.0:
            temp_c = tcal_temp_c

    # Mag (piggybacked if flag bit 0 is set)
    mag = None
    if flags & 0x01:
        mx, my, mz = struct.unpack_from("<fff", payload, 28)
        mag = (mx, my, mz)

    return {
        "seq": seq,
        "gyro": (gx, gy, gz),
        "accel": (ax, ay, az),
        "mag": mag,
        "temp_c": temp_c,
    }


def parse_raw_imu_quat(payload: bytes, meta: SensorMetadata):
    """Parse raw IMU + gyrQuat packet. Returns sample dict."""
    if len(payload) < 46:
        return None

    seq = struct.unpack_from(">H", payload, 2)[0]
    qw, qx, qy, qz = struct.unpack_from("<ffff", payload, 4)
    ax, ay, az = struct.unpack_from("<fff", payload, 20)

    flags = payload[44]
    temp_c = None
    if len(payload) >= 49:
        tcal_temp_c = struct.unpack_from("<f", payload, 45)[0]
        if math.isfinite(tcal_temp_c) and -100.0 < tcal_temp_c < 150.0:
            temp_c = tcal_temp_c

    mag = None
    if flags & 0x01:
        mx, my, mz = struct.unpack_from("<fff", payload, 32)
        mag = (mx, my, mz)

    return {
        "seq": seq,
        "gyr_quat": (qw, qx, qy, qz),
        "accel": (ax, ay, az),
        "mag": mag,
        "temp_c": temp_c,
    }


def parse_raw_mag(payload: bytes, _meta: SensorMetadata):
    """Parse legacy raw mag packet (float format)."""
    if len(payload) < 16:
        return None

    seq = struct.unpack_from(">H", payload, 2)[0]
    mx, my, mz = struct.unpack_from("<fff", payload, 4)

    return {
        "seq": seq,
        "mag": (mx, my, mz),
    }


def read_frames(port, baudrate=115200, yield_idle=False):
    """Generator yielding parsed frames; optionally emits timeout heartbeats."""
    import serial

    ser = serial.Serial(port, baudrate, timeout=0.1)
    buf = bytearray()

    try:
        while True:
            data = ser.read(256)
            if data:
                buf.extend(data)

            # Look for sync marker
            while len(buf) >= 4:
                # Find sync bytes
                idx = buf.find(b"\xAA\x55")
                if idx < 0:
                    # No sync found, keep last byte in case it's 0xAA
                    if len(buf) > 1:
                        buf = buf[-1:]
                    break

                if idx > 0:
                    buf = buf[idx:]

                if len(buf) < 3:
                    break

                frame_len = buf[2]  # payload length (includes rssi + ticks)
                total_frame = 3 + frame_len + 1  # sync(2) + len(1) + payload + crc(1)

                if len(buf) < total_frame:
                    break

                # Extract frame
                frame_data = bytes(buf[3 : 3 + frame_len])
                frame_crc = buf[3 + frame_len]

                # Verify CRC
                calc_crc = crc8_ccitt(bytes([frame_len]) + frame_data)
                if calc_crc != frame_crc:
                    # CRC mismatch, skip this sync and look for next
                    buf = buf[2:]
                    continue

                # Extract footer (rssi + rx_ticks)
                esb_len = frame_len - 5  # subtract rssi(1) + ticks(4)
                if esb_len < 2:
                    buf = buf[total_frame:]
                    continue

                esb_payload = frame_data[:esb_len]
                rssi = frame_data[esb_len]
                rx_ticks = struct.unpack_from(">I", frame_data, esb_len + 1)[0]

                yield esb_payload, rssi, rx_ticks
                buf = buf[total_frame:]

            if not data:
                if yield_idle:
                    yield None
                else:
                    time.sleep(0.001)
    finally:
        ser.close()

class TrackerCollectionState:
    """Independent collection state for one tracker stream."""

    def __init__(self, output_path, tracker_id=None):
        self.tracker_id = tracker_id
        self.control_tracker_id = tracker_id if tracker_id is not None else 0
        self.meta = SensorMetadata()
        self.cal = CalibrationData()
        self.frame_count = 0
        self.sample_count = 0
        self.last_status_samples = 0
        self.last_rssi = 0
        self.reorder = SampleReorder(self._write_sample)
        self.data_mode = "raw"
        self.meta_written = False
        self.reorder_buf = self.reorder.pending
        self.first_sample_time = None
        self.csv_file = None
        self.base = Path(output_path)
        if tracker_id is not None:
            stem = self.base.with_suffix("")
            self.base = stem.parent / f"{stem.name}.tracker-{tracker_id}"
            self.meta_path = stem.parent / f"{stem.name}.tracker-{tracker_id}.meta.txt"
            self.csv_path = stem.parent / f"{stem.name}.tracker-{tracker_id}.csv"
        else:
            self.meta_path = self.base.with_suffix(".meta.txt")
            self.csv_path = self.base.with_suffix(".csv")

    def metadata_complete(self):
        if not self.meta.received or self.cal.accel_BAinv is None or self.cal.gyro_bias is None:
            return False
        if self.meta.mag_id and self.cal.mag_BAinv is None:
            return False
        return self.cal.tcal_state_received and self.cal.tcal_complete
    def missing_metadata_request(self):
        if not self.meta.received:
            return META_MASK_BASIC, 255
        mask = 0
        if self.cal.accel_BAinv is None:
            mask |= META_MASK_ACCEL
        if self.meta.mag_id and self.cal.mag_BAinv is None:
            mask |= META_MASK_MAG
        if self.cal.gyro_bias is None:
            mask |= META_MASK_GYRO
        if not self.cal.tcal_complete or not self.cal.tcal_state_received:
            if not self.cal.tcal_state_received:
                mask |= META_MASK_TCAL_STATE
            if self.cal.tcal_num_points:
                mask |= META_MASK_TCAL_POINTS
                chunk_count = (self.cal.tcal_num_points + 1) // 2
                for chunk in range(chunk_count):
                    if chunk not in self.cal._tcal_chunks:
                        return mask, chunk
            else:
                mask |= META_MASK_TCAL_POINTS
            return mask, 255
        return (mask, 255) if mask else None

    @property
    def label(self):
        return f"tracker {self.tracker_id}" if self.tracker_id is not None else "main"
    def _csv_header(self):
        if self.data_mode == "gyr_quat":
            return "seq,qw,qx,qy,qz,ax,ay,az,mx,my,mz,temp,discontinuity\n"
        return "seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp,discontinuity\n"

    def _ensure_csv(self):
        if self.csv_file is None:
            self.csv_file = open(self.csv_path, "w", encoding="utf-8")
            self.csv_file.write(self._csv_header())

    def _switch_to_quat(self):
        if self.data_mode != "raw":
            return
        if self.sample_count or self.reorder_buf:
            raise ValueError("Cannot change raw/quat mode within one CSV capture")
        self.data_mode = "gyr_quat"
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = open(self.csv_path, "w", encoding="utf-8")
            self.csv_file.write(self._csv_header())

    def _write_metadata(self):
        if not self.meta.received:
            return
        with open(self.meta_path, "w", encoding="utf-8") as f:
            f.write(f"gyro_range_dps={self.meta.gyro_range}\n")
            f.write(f"accel_range_g={self.meta.accel_range}\n")
            f.write(f"gyro_odr_hz={self.meta.gyro_odr}\n")
            if self.meta.gyro_chip_odr is not None:
                f.write(f"gyro_chip_odr_hz={self.meta.gyro_chip_odr}\n")
            if self.meta.gyro_fusion_odr is not None:
                f.write(f"gyro_fusion_odr_hz={self.meta.gyro_fusion_odr}\n")
            f.write(f"accel_odr_hz={self.meta.accel_odr}\n")
            f.write(f"mag_odr_hz={self.meta.mag_odr}\n")
            f.write(f"imu_id={self.meta.imu_id}\n")
            f.write(f"mag_id={self.meta.mag_id}\n")
            f.write("temp_source=tcal_float_c\n")
            f.write(f"data_mode={self.data_mode}\n")
            if self.cal.has_data:
                self.cal.write_to_file(f)
        self.meta_written = True

    def _format_sample(self, sample, discontinuity=False):
        mag = sample.get("mag") or (0.0, 0.0, 0.0)
        temp = sample.get("temp_c") or 0.0
        seq = sample["seq"]
        if self.data_mode == "gyr_quat":
            q = sample["gyr_quat"]
            return (
                f"{seq},"
                f"{q[0]:.9f},{q[1]:.9f},{q[2]:.9f},{q[3]:.9f},"
                f"{sample['accel'][0]:.6f},{sample['accel'][1]:.6f},{sample['accel'][2]:.6f},"
                f"{mag[0]:.6f},{mag[1]:.6f},{mag[2]:.6f},"
                f"{temp:.6f},{int(discontinuity)}\n"
            )
        return (
            f"{seq},"
            f"{sample['gyro'][0]:.6f},{sample['gyro'][1]:.6f},{sample['gyro'][2]:.6f},"
            f"{sample['accel'][0]:.6f},{sample['accel'][1]:.6f},{sample['accel'][2]:.6f},"
            f"{mag[0]:.6f},{mag[1]:.6f},{mag[2]:.6f},"
            f"{temp:.6f},{int(discontinuity)}\n"
        )

    @property
    def gap_count(self):
        return self.reorder.gap_count

    @property
    def retransmit_count(self):
        return self.reorder.retransmit_count

    def _write_sample(self, sample, discontinuity):
        self.csv_file.write(self._format_sample(sample, discontinuity))
        self.sample_count += 1

    def handle_payload(self, payload, rssi, now):
        pkt_type = payload[0]
        if self.tracker_id is None and len(payload) >= 2:
            self.control_tracker_id = payload[1]
        self.frame_count += 1
        self.last_rssi = rssi
        if pkt_type == 0x12:
            self.meta.parse(payload)
            print(f"\nMetadata received ({self.label}): {self.meta}")
            if self.csv_file is not None:
                self._write_metadata()
            return
        if pkt_type == 0x14:
            self.cal.parse(payload)
            print(f"  {self.label}: Cal sub={payload[2]} accel={self.cal.accel_BAinv is not None} mag={self.cal.mag_BAinv is not None} gyro={self.cal.gyro_bias is not None}")
            return
        if pkt_type not in (0x10, 0x13):
            return
        if pkt_type == 0x10 and self.data_mode == "gyr_quat":
            raise ValueError("Cannot change raw/quat mode within one CSV capture")
        if pkt_type == 0x13:
            self._switch_to_quat()
        self._ensure_csv()
        self._write_metadata()
        sample = parse_raw_imu_quat(payload, self.meta) if pkt_type == 0x13 else parse_raw_imu(payload, self.meta)
        if sample is None:
            return
        if self.first_sample_time is None:
            self.first_sample_time = now
        self.reorder.push(sample, bytes(payload))

    def finalize(self, data_duration):
        self.reorder.finish()
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        if self.meta_written:
            with open(self.meta_path, "a", encoding="utf-8") as f:
                if self.cal.has_data:
                    f.write("\n# Final calibration (complete)\n")
                    self.cal.write_to_file(f)
                f.write("\n# Collection summary\n")
                f.write(f"duration_s={data_duration:.3f}\n")
                f.write(f"sample_count={self.sample_count}\n")
                f.write(f"gap_count={self.gap_count}\n")
                f.write(f"unknown_boundaries={self.reorder.unknown_boundaries}\n")


def collect(port, output_path, duration=None, batch=False, control_port=None, baudrate=115200):
    """Collect raw data, optionally maintaining independent tracker states."""
    start_time = time.time()
    last_status = start_time
    status_ticks = 0
    last_status_arrivals = 0
    states = {}
    legacy_state = TrackerCollectionState(output_path)
    if not batch:
        legacy_state._ensure_csv()
    sender = make_control_sender(
        control_port,
        receiver_serial=receiver_serial_for_cdc(port),
        baudrate=baudrate,
    )
    scheduler = MetadataRepairScheduler(sender, session_start=start_time)

    print(f"Collecting data from {port}...")
    if batch:
        print(f"Output: {Path(output_path)}.tracker-<id>.csv")
    else:
        print(f"Output: {legacy_state.csv_path}")
    if duration:
        print(f"Duration: {duration}s")
    print("Press Ctrl+C to stop\n")

    try:
        for frame in read_frames(port, baudrate, yield_idle=True):
            if frame is None:
                scheduler.service(list(states.values()) if batch else [legacy_state], time.time())
                continue
            esb_payload, rssi, rx_ticks = frame
            if not esb_payload:
                continue
            pkt_type = esb_payload[0]
            if batch and pkt_type in (0x10, 0x11, 0x12, 0x13, 0x14):
                if len(esb_payload) < 2:
                    continue
                tracker_id = esb_payload[1]
                if not 0 <= tracker_id <= 15:
                    print(f"Ignoring packet with invalid tracker ID {tracker_id}")
                    continue
                state = states.get(tracker_id)
                if state is None:
                    state = TrackerCollectionState(output_path, tracker_id)
                    states[tracker_id] = state
            else:
                state = legacy_state
            now = time.time()
            scheduler.service(list(states.values()) if batch else [legacy_state], now)
            state.handle_payload(esb_payload, rssi, now)

            if now - last_status >= 2.0:
                all_states = list(states.values()) if batch else [legacy_state]
                status_ticks += 1
                arrivals = sum(s.sample_count + len(s.reorder_buf) for s in all_states)
                sample_count = sum(s.sample_count for s in all_states)
                gap_count = sum(s.gap_count for s in all_states)
                elapsed = now - (min(
                    (s.first_sample_time for s in all_states if s.first_sample_time is not None),
                    default=start_time,
                ))
                period = now - last_status
                rate = (arrivals - last_status_arrivals) / period if period > 0 else 0
                buffered = sum(len(s.reorder_buf) for s in all_states)
                total = sample_count + gap_count
                loss_pct = gap_count / total * 100 if total > 0 else 0
                print(
                    f"\r[{elapsed:.1f}s] in:{rate:.0f}/s ok:{sample_count} "
                    f"loss:{gap_count} ({loss_pct:.1f}%) buf:{buffered} "
                    f"rssi:{max((s.last_rssi for s in all_states), default=0)} "
                    f"trackers:{len(all_states)}   ",
                    end="",
                    flush=True,
                )
                if batch and status_ticks % 5 == 0:
                    detail = "  ".join(
                        f"t{item.tracker_id}:"
                        f"{(item.sample_count + len(item.reorder_buf) - item.last_status_samples) / period:.0f}/s"
                        f" ok={item.sample_count} loss={item.gap_count}"
                        for item in all_states
                    )
                    print(f"\n  {detail}", flush=True)
                for item in all_states:
                    if item.csv_file is not None:
                        item.csv_file.flush()
                    item.last_status_samples = item.sample_count + len(item.reorder_buf)
                last_status_arrivals = arrivals
                last_status = now

            first_sample = min(
                (s.first_sample_time for s in states.values() if s.first_sample_time is not None),
                default=legacy_state.first_sample_time,
            ) if batch else legacy_state.first_sample_time
            if duration and first_sample and now - first_sample >= duration:
                break
    except KeyboardInterrupt:
        print("\n\nStopping collection...")
    finally:
        all_states = list(states.values()) if batch else [legacy_state]
        first_sample = min(
            (s.first_sample_time for s in all_states if s.first_sample_time is not None),
            default=None,
        )
        end_time = time.time()
        data_duration = end_time - first_sample if first_sample else 0
        for state in all_states:
            state._write_metadata()
            state_duration = end_time - state.first_sample_time if state.first_sample_time else 0
            state.finalize(state_duration)
        scheduler.close()

    sample_count = sum(s.sample_count for s in all_states)
    gap_count = sum(s.gap_count for s in all_states)
    retransmit_count = sum(s.retransmit_count for s in all_states)
    total = sample_count + gap_count
    loss_pct = gap_count / total * 100 if total > 0 else 0
    print("\n\nCollection complete:")
    print(f"  Duration: {data_duration:.1f}s")
    print(f"  Samples: {sample_count}")
    print(f"  Retransmits received: {retransmit_count}")
    print(f"  Gaps (lost): {gap_count} ({loss_pct:.2f}%)")
    print(f"  Unknown boundaries: {sum(s.reorder.unknown_boundaries for s in all_states)}")
    for state in all_states:
        if batch:
            print(f"  Tracker-{state.tracker_id}: {state.sample_count} samples -> {state.csv_path}")
        else:
            print(f"  Samples: {state.sample_count} -> {state.csv_path}")
        if state.meta_written:
            print(f"  Metadata: {state.meta_path}")
            print(f"  Data mode: {state.data_mode}")

def find_data_port():
    """Find SlimeNRF CDC ports and list them for the user.

    With dual CDC ACM, the receiver enumerates two serial ports:
      - Lower interface/port number = console
      - Higher interface/port number = data collection

    Returns the device path if only one port found, otherwise None.
    """
    from serial.tools import list_ports

    candidates = []
    for p in list_ports.comports():
        if p.vid == SLIME_VID and p.pid == SLIME_PID:
            candidates.append(p)

    if not candidates:
        return None

    if len(candidates) == 1:
        print(f"Found one SlimeNRF port: {candidates[0].device}")
        return candidates[0].device

    # Sort by interface number or device path
    candidates.sort(key=lambda p: (getattr(p, 'interface_number', None) or 0, p.device))
    print("Found multiple SlimeNRF CDC ports:")
    for i, p in enumerate(candidates):
        iface = getattr(p, 'interface_number', '?')
        print(f"  [{i}] {p.device}  (interface={iface}, {p.description})")
    print("\nData port is usually the higher-numbered port.")
    print(f"Specify the port manually, e.g.: data_collect.py {candidates[-1].device}")
    return None


def main():
    parser = argparse.ArgumentParser(
        description="SlimeVR Raw Sensor Data Collector"
    )
    parser.add_argument(
        "port",
        nargs="?",
        default=None,
        help="CDC ACM serial port for data (e.g., /dev/ttyACM1). "
             "Auto-detected if omitted.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="sensor_data",
        help="Output file base name (default: sensor_data)",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=None,
        help="Collection duration in seconds (default: until Ctrl+C)",
    )
    parser.add_argument(
        "-b",
        "--baudrate",
        type=int,
        default=115200,
        help="Serial baudrate (default: 115200, CDC ACM ignores this)",
    )
    parser.add_argument(
        "--batch",
        action="store_true",
        help="Collect independent output files for each tracker ID",
    )
    parser.add_argument(
        "--control-port",
        default=None,
        help="CDC console port for collectmeta requests; otherwise use serial-matched HID control.",
    )

    args = parser.parse_args()

    try:
        import serial  # noqa: F401
    except ImportError:
        print("Error: pyserial is required. Install with: pip install pyserial")
        sys.exit(1)

    port = args.port
    if port is None:
        port = find_data_port()
        if port is None:
            print("Error: No SlimeNRF receiver found. "
                  "Specify port manually, e.g.: data_collect.py /dev/ttyACM1")
            sys.exit(1)
    collect(
        port,
        args.output,
        args.duration,
        batch=args.batch,
        control_port=args.control_port,
        baudrate=args.baudrate,
    )



if __name__ == "__main__":
    main()
