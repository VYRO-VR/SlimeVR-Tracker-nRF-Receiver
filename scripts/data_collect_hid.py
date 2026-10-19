#!/usr/bin/env python
# /// script
# dependencies = [
#   "hidapi",
# ]
# ///

"""
SlimeVR Raw Sensor Data Collector (HID version)

Reads raw sensor data from the SlimeVR receiver's dedicated HID endpoint
and saves it to CSV for offline VQF parameter optimization.

HID report format (64 bytes):
  [0..N-1]    ESB payload (same format as over-the-air)
  [N]         RSSI
  [N+1..N+4]  rx_ticks (BE32, Zephyr uptime ticks)
  [N+5..63]   zero padding

Packet types in ESB payload (byte 0):
  0x10: Raw IMU (48 bytes) - float gyro+accel (+optional body-frame aligned mag) + T-Cal temp
  0x11: Raw Mag (17 bytes) - float body-frame aligned magnetometer
  meta: raw TX Hz, optional chip/fusion Hz after mag_id

Output: CSV columns end in temp,discontinuity; gyrQuat uses qw,qx,qy,qz
instead of gx,gy,gz. A discontinuity marks unknown loss, not proven reboot.
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
)

# SlimeNRF Receiver USB IDs
SLIME_VID = 0x1209
SLIME_PID = 0x7690

# HID interface: vendor usage page 0xFF00 (data collection endpoint)
DC_USAGE_PAGE = 0xFF00


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


def parse_raw_imu(payload: bytes):
    """Parse legacy raw IMU packet. Returns sample dict."""
    if len(payload) < 42:
        return None

    seq = struct.unpack_from(">H", payload, 2)[0]
    gx, gy, gz = struct.unpack_from("<fff", payload, 4)
    ax, ay, az = struct.unpack_from("<fff", payload, 16)

    flags = payload[40]
    temp_c = None
    if len(payload) >= 45:
        tcal_temp_c = struct.unpack_from("<f", payload, 41)[0]
        if math.isfinite(tcal_temp_c) and -100.0 < tcal_temp_c < 150.0:
            temp_c = tcal_temp_c

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


def parse_raw_imu_quat(payload: bytes):
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


def find_data_hid(device_index=None):
    """Find the SlimeNRF data collection HID interface.

    The receiver has two HID interfaces:
      - Interface with standard usage page: tracker data
      - Interface with vendor usage page 0xFF00: data collection

    If multiple receivers are connected, lists them and requires
    device_index to select one.

    Returns (path, info_dict) or (None, None).
    """
    import hid

    devices = hid.enumerate(SLIME_VID, SLIME_PID)
    if not devices:
        return None, None

    # Filter for vendor-defined data collection interfaces
    dc_devices = [d for d in devices if d.get("usage_page") == DC_USAGE_PAGE]

    if not dc_devices:
        # Debug: show what was enumerated
        print("No device with vendor usage page 0xFF00 found.")
        print("Enumerated HID interfaces:")
        for d in devices:
            print(f"  interface={d.get('interface_number', '?')}, "
                  f"usage_page=0x{d.get('usage_page', 0):04X}, "
                  f"usage=0x{d.get('usage', 0):04X}")
        return None, None

    if len(dc_devices) == 1 and device_index is None:
        return dc_devices[0]["path"], dc_devices[0]

    if len(dc_devices) > 1:
        print(f"Found {len(dc_devices)} SlimeNRF data collection devices:")
        for i, d in enumerate(dc_devices):
            sn = d.get("serial_number", "?")
            mfr = d.get("manufacturer_string", "?")
            iface = d.get("interface_number", "?")
            print(f"  [{i}] SN={sn}  MFR={mfr}  interface={iface}")

        if device_index is None:
            print("\nUse --device N to select a device.")
            return None, None

    idx = device_index if device_index is not None else 0
    if idx >= len(dc_devices):
        print(f"Error: device index {idx} out of range (0-{len(dc_devices)-1})")
        return None, None

    return dc_devices[idx]["path"], dc_devices[idx]


class _CollectorState:
    """Independent metadata, calibration, output, and sequence state."""

    def __init__(self, output_path, tracker_id=None):
        self.tracker_id = tracker_id
        self.control_tracker_id = tracker_id if tracker_id is not None else 0
        self.meta = SensorMetadata()
        self.cal = CalibrationData()
        self.first_sample_time = None
        self.frame_count = 0
        self.sample_count = 0
        self.last_rssi = 0
        self.reorder = SampleReorder(self._write_sample)
        self.meta_written = False
        self.raw_seen = False
        self.reorder_buf = self.reorder.pending
        self.csv_file = None
        self.data_mode = "raw"
        base = Path(output_path)
        if tracker_id is not None:
            stem = base.with_suffix("")
            self.csv_path = stem.parent / f"{stem.name}.tracker-{tracker_id}.csv"
            self.meta_path = stem.parent / f"{stem.name}.tracker-{tracker_id}.meta.txt"
        else:
            self.csv_path = base.with_suffix(".csv")
            self.meta_path = base.with_suffix(".meta.txt")
    @property
    def label(self):
        return f"tracker {self.tracker_id}" if self.tracker_id is not None else "legacy"

    def _open_csv(self, data_mode):
        if self.csv_file is not None:
            self.csv_file.close()
        self.data_mode = data_mode
        self.csv_file = open(self.csv_path, "w", encoding="utf-8")
        header = "seq,qw,qx,qy,qz,ax,ay,az,mx,my,mz,temp,discontinuity\n" if data_mode == "gyr_quat" else "seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp,discontinuity\n"
        self.csv_file.write(header)

    def parse_metadata(self, payload):
        self.meta.parse(payload)
        print(f"\nMetadata received ({self.label}): {self.meta}")
        if self.csv_file is not None:
            self.write_metadata()

    def parse_calibration(self, payload):
        self.cal.parse(payload)
        print(f"  {self.label}: Cal sub={payload[2]} accel={self.cal.accel_BAinv is not None} mag={self.cal.mag_BAinv is not None} gyro={self.cal.gyro_bias is not None}")
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
                for chunk in range((self.cal.tcal_num_points + 1) // 2):
                    if chunk not in self.cal._tcal_chunks:
                        return mask, chunk
            else:
                mask |= META_MASK_TCAL_POINTS
            return mask, 255
        return (mask, 255) if mask else None

    def ensure_csv(self, pkt_type):
        mode = "gyr_quat" if pkt_type == 0x13 else "raw"
        if mode != self.data_mode and (self.sample_count or self.reorder_buf):
            raise ValueError("Cannot change raw/quat mode within one CSV capture")
        if self.csv_file is None:
            self._open_csv("gyr_quat" if pkt_type == 0x13 else "raw")
        elif pkt_type == 0x13 and self.data_mode == "raw":
            # The initial empty raw header may be replaced before any sample.
            self._open_csv("gyr_quat")
            if self.meta_written:
                # Metadata is append-only after first raw data; rewrite its
                # mode line so delayed gyrQuat packets remain self-describing.
                lines = self.meta_path.read_text(encoding="utf-8").splitlines()
                with open(self.meta_path, "w", encoding="utf-8") as f:
                    for line in lines:
                        f.write("data_mode=gyr_quat\n" if line.startswith("data_mode=") else line + "\n")

    def write_metadata(self):
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


    @property
    def gap_count(self):
        return self.reorder.gap_count

    @property
    def retransmit_count(self):
        return self.reorder.retransmit_count

    def _write_sample(self, sample, discontinuity):
        self.csv_file.write(self._csv_line(sample, sample["seq"], discontinuity))
        self.sample_count += 1

    def accept_raw(self, payload, pkt_type, now):
        sample = parse_raw_imu_quat(payload) if pkt_type == 0x13 else parse_raw_imu(payload)
        if not sample:
            return False

        self.ensure_csv(pkt_type)
        if self.meta.received and not self.meta_written:
            self.write_metadata()

        if self.first_sample_time is None:
            self.first_sample_time = now
            self.raw_seen = True
        return self.reorder.push(sample, bytes(payload))

    def _csv_line(self, sample, seq, discontinuity=False):
        mag = sample.get("mag") or (0.0, 0.0, 0.0)
        temp = sample.get("temp_c") or 0.0
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

    def flush(self):
        if self.csv_file is not None:
            self.csv_file.flush()

    def finalize(self, end_time):
        self.reorder.finish()
        self.flush()
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        if not self.raw_seen:
            return 0.0
        data_duration = end_time - self.first_sample_time if self.first_sample_time else 0.0
        if not self.meta_written and self.meta.received:
            self.write_metadata()
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
        return data_duration

def collect_hid(output_path, duration=None, device_index=None, batch=False, control_port=None):
    """Collect HID raw packets, optionally maintaining one state per tracker."""
    import hid

    dev_path, dev_info = find_data_hid(device_index)
    if dev_path is None:
        print("Error: No SlimeNRF data collection HID endpoint found.")
        print("Make sure the receiver firmware was built with CONFIG_DATA_COLLECT=y")
        sys.exit(1)

    iface = dev_info.get("interface_number", "?")
    print(f"Found data HID: interface={iface}, usage_page=0x{dev_info.get('usage_page', 0):04X}")
    h = hid.device()
    h.open_path(dev_path)
    sender = make_control_sender(
        control_port,
        receiver_serial=dev_info.get("serial_number"),
        data_hid_path=dev_path,
    )
    scheduler = MetadataRepairScheduler(sender)
    start_time = time.time()
    scheduler.session_start = start_time
    first_sample_time = None
    frame_count = 0
    last_status = start_time
    status_ticks = 0
    last_status_arrivals = 0
    states = {}
    legacy_state = None if batch else _CollectorState(output_path)

    print("Collecting data via HID...")
    if batch:
        print(f"Output: {Path(output_path)}.tracker-<id>.csv")
    else:
        print(f"Output: {legacy_state.csv_path}")
    if duration:
        print(f"Duration: {duration}s")
    print("Press Ctrl+C to stop\n")
    if not batch:
        legacy_state._open_csv("raw")


    def get_state(tracker_id):
        if not batch:
            return legacy_state
        state = states.get(tracker_id)
        if state is None:
            state = _CollectorState(output_path, tracker_id)
            states[tracker_id] = state
        return state

    try:
        while True:
            report = h.read(64, timeout_ms=500)
            now = time.time()
            scheduler.service(list(states.values()) if batch else [legacy_state], now)
            if not report:
                continue
            report = bytes(report)
            if len(report) < 2:
                continue
            pkt_type = report[0]
            if pkt_type == 0x10:
                esb_len = 48
            elif pkt_type == 0x13:
                esb_len = 52
            elif pkt_type == 0x11:
                esb_len = 16
            elif pkt_type in (0x12, 0x14):
                esb_len = 52
            else:
                continue
            if len(report) < esb_len:
                continue
            esb_payload = report[:esb_len]
            tracker_id = esb_payload[1]
            if batch and not 0 <= tracker_id <= 15:
                continue
            state = get_state(tracker_id)
            state.frame_count += 1
            frame_count += 1
            if not batch:
                legacy_state.control_tracker_id = tracker_id
            state = get_state(tracker_id)

            if pkt_type == 0x12:
                state.parse_metadata(esb_payload)
            elif pkt_type == 0x14:
                state.parse_calibration(esb_payload)
            elif pkt_type == 0x10 or pkt_type == 0x13:
                now = time.time()
                accepted = state.accept_raw(esb_payload, pkt_type, now)
                if accepted and first_sample_time is None:
                    first_sample_time = now

            now = time.time()
            if now - last_status >= 2.0:
                current_states = states.values() if batch else (legacy_state,)
                for current in current_states:
                    current.flush()
                all_states = list(states.values()) if batch else [legacy_state]
                status_ticks += 1
                arrivals = sum(s.sample_count + len(s.reorder_buf) for s in all_states)
                aggregate_samples = sum(s.sample_count for s in all_states)
                aggregate_gaps = sum(s.gap_count for s in all_states)
                period = now - last_status
                rate = ((arrivals - last_status_arrivals) / period
                        if period > 0 else 0)
                buffered = sum(len(s.reorder_buf) for s in all_states)
                total = aggregate_samples + aggregate_gaps
                loss_pct = aggregate_gaps / total * 100 if total > 0 else 0
                print(
                    f"\r[{now - (first_sample_time or start_time):.1f}s] "
                    f"in:{rate:.0f}/s ok:{aggregate_samples} "
                    f"loss:{aggregate_gaps} ({loss_pct:.1f}%) buf:{buffered} "
                    f"trackers:{len(all_states)}   ",
                    end="", flush=True,
                )
                if batch and status_ticks % 5 == 0:
                    detail = "  ".join(
                        f"t{item.tracker_id}:"
                        f"{(item.sample_count + len(item.reorder_buf) - item.last_status_samples) / period:.0f}/s"
                        f" ok={item.sample_count} loss={item.gap_count}"
                        for item in all_states
                    )
                    print(f"\n  {detail}", flush=True)
                last_status = now
                last_status_arrivals = arrivals
                for item in all_states:
                    item.last_status_samples = item.sample_count + len(item.reorder_buf)

            if duration and first_sample_time and now - first_sample_time >= duration:
                break

    except KeyboardInterrupt:
        print("\n\nStopping collection...")
    finally:
        try:
            h.close()
        finally:
            end_time = time.time()
            all_states = list(states.values()) if batch else [legacy_state]
            for state in all_states:
                state.write_metadata() if state.meta.received else None
                state.finalize(end_time)
            scheduler.close()
    data_duration = time.time() - first_sample_time if first_sample_time else 0
    aggregate_samples = sum(s.sample_count for s in all_states)
    aggregate_retx = sum(s.retransmit_count for s in all_states)
    aggregate_gaps = sum(s.gap_count for s in all_states)
    total = aggregate_samples + aggregate_gaps
    loss_pct = aggregate_gaps / total * 100 if total > 0 else 0
    print("\n\nCollection complete:")
    print(f"  Duration: {data_duration:.1f}s")
    print(f"  Samples: {aggregate_samples}")
    print(f"  Retransmits received: {aggregate_retx}")
    print(f"  Gaps (lost): {aggregate_gaps} ({loss_pct:.2f}%)")
    print(f"  Unknown boundaries: {sum(s.reorder.unknown_boundaries for s in all_states)}")
    for state in all_states:
        if state.raw_seen:
            print(f"  {state.label}: {state.sample_count} samples -> {state.csv_path}")
            if state.meta_written:
                print(f"  {state.label} metadata: {state.meta_path}")


def main():
    parser = argparse.ArgumentParser(
        description="SlimeVR Raw Sensor Data Collector (HID)"
    )
    parser.add_argument(
        "-o", "--output", default="sensor_data",
        help="Output file base name (default: sensor_data)",
    )
    parser.add_argument(
        "-d", "--duration", type=float, default=None,
        help="Collection duration in seconds (default: until Ctrl+C)",
    )
    parser.add_argument(
        "--device", type=int, default=None,
        help="Device index when multiple receivers are connected (0-based)",
    )
    parser.add_argument(
        "--batch", action="store_true",
        help="Write independent output files for each tracker ID (0-15)",
    )
    parser.add_argument(
        "--control-port",
        default=None,
        help="CDC console port for collectmeta requests; otherwise use serial-matched HID control.",
    )
    args = parser.parse_args()

    try:
        import hid  # noqa: F401
    except ImportError:
        print("Error: hidapi is required. Install with: pip install hidapi")
        sys.exit(1)

    collect_hid(
        args.output,
        args.duration,
        args.device,
        args.batch,
        control_port=args.control_port,
    )



if __name__ == "__main__":
    main()
