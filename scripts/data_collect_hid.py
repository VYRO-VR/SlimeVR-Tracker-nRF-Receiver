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
  0x12: Metadata (48 bytes) - ODR, range, sensor IDs

Output: CSV file with columns: seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp
"""

import argparse
import math
import struct
import sys
import time
from pathlib import Path

# SlimeNRF Receiver USB IDs
SLIME_VID = 0x1209
SLIME_PID = 0x7690

# HID interface: vendor usage page 0xFF00 (data collection endpoint)
DC_USAGE_PAGE = 0xFF00


class SensorMetadata:
    """Stores metadata from type 0x12 packet."""

    def __init__(self):
        self.gyro_range = 0.0
        self.accel_range = 0.0
        self.gyro_odr = 0.0
        self.accel_odr = 0.0
        self.mag_odr = 0.0
        self.imu_id = 0
        self.mag_id = 0
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
        self.received = True

    def __str__(self):
        return (
            f"Gyro: {self.gyro_range:.0f} dps @ {self.gyro_odr:.0f} Hz, "
            f"Accel: {self.accel_range:.0f} g @ {self.accel_odr:.0f} Hz, "
            f"Mag: {self.mag_odr:.0f} Hz, "
            f"IMU ID: {self.imu_id}, Mag ID: {self.mag_id}"
        )


class CalibrationData:
    """Stores calibration data from type 0x14 packets."""

    def __init__(self):
        self.accel_BAinv = None   # 4×3 matrix (list of 12 floats)
        self.mag_BAinv = None     # 4×3 matrix (list of 12 floats)
        self.gyro_bias = None     # [3] floats (deg/s)
        self.gyro_sens_scale = None  # [3] floats
        self.tcal_enabled = False
        self.tcal_num_points = 0
        self.tcal_temp_min = 0.0
        self.tcal_temp_max = 0.0
        self.tcal_correction_offset = None  # [3] floats
        self.tcal_points = []     # list of (temp, bx, by, bz) tuples
        self._tcal_pending_points = []
        self._tcal_pending_chunk_idx = -1

    @staticmethod
    def _is_valid_tcal_point(point):
        temp, bx, by, bz = point
        if not all(math.isfinite(v) for v in point):
            return False
        return not (temp == 0.0 and bx == 0.0 and by == 0.0 and bz == 0.0)

    @classmethod
    def _normalize_tcal_points(cls, points):
        deduped = {}
        for point in points:
            if not cls._is_valid_tcal_point(point):
                continue
            deduped[f"{point[0]:.2f}"] = point
        return [deduped[key] for key in sorted(deduped, key=lambda k: float(k))]

    def parse(self, payload: bytes):
        """Parse a type 0x14 calibration packet. Sub-type at byte[2]."""
        if len(payload) < 4:
            return
        sub_type = payload[2]
        if sub_type == 0x01:  # ACCEL_CAL
            if len(payload) >= 51:  # 3 + 48
                self.accel_BAinv = list(struct.unpack_from("<12f", payload, 3))
                print(f"  Calibration: accel BAinv received")
        elif sub_type == 0x02:  # MAG_CAL
            if len(payload) >= 51:
                self.mag_BAinv = list(struct.unpack_from("<12f", payload, 3))
                print(f"  Calibration: mag BAinv received")
        elif sub_type == 0x03:  # GYRO_CAL
            if len(payload) >= 27:  # 3 + 24
                self.gyro_bias = list(struct.unpack_from("<3f", payload, 3))
                self.gyro_sens_scale = list(struct.unpack_from("<3f", payload, 15))
                print(f"  Calibration: gyro bias + sens received")
        elif sub_type == 0x04:  # TCAL_STATE
            if len(payload) >= 26:
                self.tcal_enabled = bool(payload[3])
                self.tcal_num_points = struct.unpack_from("<H", payload, 4)[0]
                self.tcal_temp_min = struct.unpack_from("<f", payload, 6)[0]
                self.tcal_temp_max = struct.unpack_from("<f", payload, 10)[0]
                self.tcal_correction_offset = list(
                    struct.unpack_from("<3f", payload, 14)
                )
                print(
                    f"  Calibration: T-Cal state (enabled={self.tcal_enabled}, "
                    f"points={self.tcal_num_points}, "
                    f"range={self.tcal_temp_min:.0f}-{self.tcal_temp_max:.0f}°C)"
                )
                if self.tcal_num_points == 0:
                    self.tcal_points = []
                    self._tcal_pending_points = []
                    self._tcal_pending_chunk_idx = -1
        elif sub_type == 0x05:  # TCAL_POINTS
            if len(payload) >= 23:
                # [3] chunk_idx [4-5] total_count [6] num_in_chunk
                # [7-22] point[0] [23-38] point[1]
                chunk_idx = payload[3]
                total_count = struct.unpack_from("<H", payload, 4)[0]
                num_in_chunk = payload[6]
                last_chunk_idx = ((total_count - 1) // 2) if total_count > 0 else 0
                if chunk_idx == 0 or self._tcal_pending_chunk_idx >= chunk_idx:
                    self._tcal_pending_points = []
                for j in range(num_in_chunk):
                    offset = 7 + j * 16
                    if offset + 16 <= len(payload):
                        temp, bx, by, bz = struct.unpack_from("<4f", payload, offset)
                        self._tcal_pending_points.append((temp, bx, by, bz))
                self._tcal_pending_chunk_idx = chunk_idx
                normalized = self._normalize_tcal_points(self._tcal_pending_points)
                if not self.tcal_points:
                    self.tcal_points = normalized
                    self.tcal_num_points = len(self.tcal_points)
                if chunk_idx >= last_chunk_idx:
                    self.tcal_points = normalized
                    self._tcal_pending_points = []
                    self._tcal_pending_chunk_idx = -1
                    self.tcal_num_points = len(self.tcal_points)
                if len(normalized) <= 2:  # Print only on first chunk
                    print(f"  Calibration: T-Cal points receiving...")

    def write_to_file(self, f):
        """Append calibration data to metadata file (vqf_core.py-compatible format)."""
        self.tcal_points = self._normalize_tcal_points(self.tcal_points)
        self.tcal_num_points = len(self.tcal_points)
        f.write("\n# Calibration data (from tracker retained memory)\n")
        # Accel: split BAinv[12] → bias (row 0) + matrix (rows 1-3)
        if self.accel_BAinv is not None and len(self.accel_BAinv) == 12:
            f.write(f"acc_cal_bias={','.join(f'{v:.9g}' for v in self.accel_BAinv[:3])}\n")
            f.write(f"acc_cal_matrix={','.join(f'{v:.9g}' for v in self.accel_BAinv[3:])}\n")
        # Mag: split BAinv[12] → bias (row 0) + matrix (rows 1-3)
        if self.mag_BAinv is not None and len(self.mag_BAinv) == 12:
            f.write(f"mag_cal_bias={','.join(f'{v:.9g}' for v in self.mag_BAinv[:3])}\n")
            f.write(f"mag_cal_matrix={','.join(f'{v:.9g}' for v in self.mag_BAinv[3:])}\n")
        if self.gyro_bias is not None:
            f.write(f"gyro_bias={','.join(f'{v:.9g}' for v in self.gyro_bias)}\n")
        if self.gyro_sens_scale is not None:
            f.write(f"gyro_sens_scale={','.join(f'{v:.9g}' for v in self.gyro_sens_scale)}\n")
        # T-Cal: compact gyro_tcal format (temp:bx,by,bz;temp:bx,by,bz;...)
        if self.tcal_points:
            entries = [f"{t:.2f}:{bx:.5f},{by:.5f},{bz:.5f}"
                       for t, bx, by, bz in self.tcal_points]
            f.write(f"gyro_tcal={';'.join(entries)}\n")

    @property
    def has_data(self):
        return any([
            self.accel_BAinv, self.mag_BAinv,
            self.gyro_bias, self.tcal_enabled, self.tcal_points
        ])


def parse_raw_imu(payload: bytes):
    """Parse type 0x10 raw IMU packet. Returns sample dict."""
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
    """Parse type 0x13 raw IMU + gyrQuat packet (52 bytes). Returns sample dict."""
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


def collect_hid(output_path, duration=None, device_index=None):
    """Main HID-based data collection loop with reorder buffer for ARQ retransmits."""
    import hid

    dev_path, dev_info = find_data_hid(device_index)
    if dev_path is None:
        print("Error: No SlimeNRF data collection HID endpoint found.")
        print("Make sure the receiver firmware was built with CONFIG_DATA_COLLECT=y")
        sys.exit(1)

    iface = dev_info.get("interface_number", "?")
    print(f"Found data HID: interface={iface}, "
          f"usage_page=0x{dev_info.get('usage_page', 0):04X}")

    h = hid.device()
    h.open_path(dev_path)

    meta = SensorMetadata()
    cal = CalibrationData()
    start_time = time.time()
    first_sample_time = None  # set when first data sample arrives
    frame_count = 0
    sample_count = 0
    last_status = start_time
    last_status_samples = 0
    last_rssi = 0
    retransmit_count = 0  # packets that filled a gap in the reorder buffer
    gap_count = 0  # sequence gaps that were never filled (actual loss)
    meta_written = False  # metadata file written once on first IMU data

    # Reorder buffer: holds samples until we can write them in order.
    # Keyed by sequence number; flushed when contiguous run available.
    REORDER_BUF_MAX = 200  # match receiver raw ARQ stale window before force-flush
    reorder_buf = {}  # seq -> csv_line
    write_cursor = None  # next seq expected to be written

    base = Path(output_path)
    meta_path = base.with_suffix(".meta.txt")
    csv_path = base.with_suffix(".csv")

    print(f"Collecting data via HID...")
    print(f"Output: {csv_path}")
    if duration:
        print(f"Duration: {duration}s")
    print("Press Ctrl+C to stop\n")

    csv_file = open(csv_path, "w", encoding="utf-8")
    csv_file.write("seq,gx,gy,gz,ax,ay,az,mx,my,mz,temp\n")
    data_mode = "raw"  # will switch to "gyr_quat" if type 0x13 packets arrive

    def flush_reorder_buf():
        """Write contiguous samples from write_cursor onwards."""
        nonlocal write_cursor, sample_count
        if write_cursor is None:
            return
        while write_cursor in reorder_buf:
            csv_file.write(reorder_buf.pop(write_cursor))
            sample_count += 1
            write_cursor = (write_cursor + 1) & 0xFFFF

    def force_flush_reorder_buf():
        """Force-flush oldest samples when buffer is too large."""
        nonlocal write_cursor, sample_count, gap_count
        if not reorder_buf:
            return
        # Sort remaining keys and write them all, counting gaps
        sorted_seqs = sorted(reorder_buf.keys(),
                             key=lambda s: (s - write_cursor) & 0xFFFF)
        for seq in sorted_seqs:
            # Count gaps between write_cursor and this seq
            gap = (seq - write_cursor) & 0xFFFF
            if gap > 0:
                gap_count += gap
                write_cursor = seq
            csv_file.write(reorder_buf[seq])
            sample_count += 1
            write_cursor = (write_cursor + 1) & 0xFFFF
        reorder_buf.clear()

    try:
        while True:
            report = h.read(64, timeout_ms=500)
            if not report:
                continue

            report = bytes(report)
            if len(report) < 2:
                continue

            pkt_type = report[0]
            frame_count += 1

            if pkt_type == 0x10:
                esb_len = 48
            elif pkt_type == 0x13:
                esb_len = 52
            elif pkt_type == 0x11:
                esb_len = 16
            elif pkt_type == 0x12:
                esb_len = 48
            elif pkt_type == 0x14:
                esb_len = 52
            else:
                continue

            esb_payload = report[:esb_len]
            if len(report) > esb_len:
                last_rssi = report[esb_len]

            if pkt_type == 0x12:  # Metadata
                meta.parse(esb_payload)
                print(f"\nMetadata received: {meta}")

            elif pkt_type == 0x14:  # Calibration data
                cal.parse(esb_payload)
                print(f"  Cal sub={esb_payload[2]} accel={cal.accel_BAinv is not None} mag={cal.mag_BAinv is not None} gyro={cal.gyro_bias is not None}")

            elif pkt_type == 0x10 or pkt_type == 0x13:  # Raw IMU or gyrQuat
                # Detect mode on first packet
                if pkt_type == 0x13 and data_mode == "raw":
                    data_mode = "gyr_quat"
                    csv_file.close()
                    csv_file = open(csv_path, "w", encoding="utf-8")
                    csv_file.write("seq,qw,qx,qy,qz,ax,ay,az,mx,my,mz,temp\n")

                # Write metadata file once on first IMU data arrival
                if meta.received and not meta_written:
                    meta_written = True
                    with open(meta_path, "w", encoding="utf-8") as f:
                        f.write(f"gyro_range_dps={meta.gyro_range}\n")
                        f.write(f"accel_range_g={meta.accel_range}\n")
                        f.write(f"gyro_odr_hz={meta.gyro_odr}\n")
                        f.write(f"accel_odr_hz={meta.accel_odr}\n")
                        f.write(f"mag_odr_hz={meta.mag_odr}\n")
                        f.write(f"imu_id={meta.imu_id}\n")
                        f.write(f"mag_id={meta.mag_id}\n")
                        f.write("temp_source=tcal_float_c\n")
                        f.write(f"data_mode={data_mode}\n")
                        if cal.has_data:
                            cal.write_to_file(f)
                    print(f"  Metadata + calibration written to {meta_path}")

                if pkt_type == 0x13:
                    s = parse_raw_imu_quat(esb_payload)
                else:
                    s = parse_raw_imu(esb_payload)

                if s:
                    seq = s["seq"]

                    # Detect retransmit (seq < write_cursor or already in buffer)
                    if write_cursor is not None:
                        diff = (seq - write_cursor) & 0xFFFF
                        if diff > 0x8000:
                            # seq is behind write_cursor — late retransmit, already written
                            retransmit_count += 1
                            continue
                    if seq in reorder_buf:
                        # Duplicate
                        continue

                    # Initialize write cursor on first sample
                    if write_cursor is None:
                        write_cursor = seq
                        first_sample_time = time.time()

                    mag = s.get("mag") or (0.0, 0.0, 0.0)
                    temp = s.get("temp_c") or 0.0

                    if data_mode == "gyr_quat":
                        q = s["gyr_quat"]
                        csv_line = (
                            f"{seq},"
                            f"{q[0]:.9f},{q[1]:.9f},{q[2]:.9f},{q[3]:.9f},"
                            f"{s['accel'][0]:.6f},{s['accel'][1]:.6f},{s['accel'][2]:.6f},"
                            f"{mag[0]:.6f},{mag[1]:.6f},{mag[2]:.6f},"
                            f"{temp:.6f}\n"
                        )
                    else:
                        csv_line = (
                            f"{seq},"
                            f"{s['gyro'][0]:.6f},{s['gyro'][1]:.6f},{s['gyro'][2]:.6f},"
                            f"{s['accel'][0]:.6f},{s['accel'][1]:.6f},{s['accel'][2]:.6f},"
                            f"{mag[0]:.6f},{mag[1]:.6f},{mag[2]:.6f},"
                            f"{temp:.6f}\n"
                        )

                    reorder_buf[seq] = csv_line

                    # Detect retransmit: a packet that fills a gap
                    # (write_cursor < seq, but seq is behind some already-buffered seqs)
                    diff = (seq - write_cursor) & 0xFFFF
                    if diff > 0 and diff < REORDER_BUF_MAX:
                        # Check if there are buffered seqs ahead of this one
                        # (meaning this seq was missing and just got retransmitted)
                        has_later = any(
                            0 < ((s2 - seq) & 0xFFFF) < REORDER_BUF_MAX
                            for s2 in reorder_buf if s2 != seq
                        )
                        if has_later:
                            retransmit_count += 1

                    # Flush contiguous samples from write cursor
                    flush_reorder_buf()

                    # Force flush if buffer is too large (gap not filled in time)
                    if len(reorder_buf) >= REORDER_BUF_MAX:
                        force_flush_reorder_buf()

            now = time.time()
            if now - last_status >= 2.0:
                csv_file.flush()
                elapsed = now - (first_sample_time or start_time)
                period = now - last_status
                period_samples = sample_count - last_status_samples
                rate = period_samples / period if period > 0 else 0
                buffered = len(reorder_buf)
                retx_info = f", Retx: {retransmit_count}" if retransmit_count > 0 else ""
                buf_info = f", Buf: {buffered}" if buffered > 0 else ""
                # Loss = gaps that were never filled
                total = sample_count + gap_count
                loss_pct = gap_count / total * 100 if total > 0 else 0
                loss_info = f", Loss: {gap_count} ({loss_pct:.1f}%)" if gap_count > 0 else ""
                print(
                    f"\r[{elapsed:.1f}s] Samples: {sample_count} ({rate:.0f}/s)"
                    f"{retx_info}{buf_info}{loss_info}, RSSI: {last_rssi}   ",
                    end="",
                    flush=True,
                )
                last_status = now
                last_status_samples = sample_count

            if duration and first_sample_time and (now - first_sample_time >= duration):
                break

    except KeyboardInterrupt:
        print("\n\nStopping collection...")
    finally:
        # Flush remaining buffer
        if reorder_buf:
            force_flush_reorder_buf()
        csv_file.close()
        h.close()

    data_duration = (time.time() - first_sample_time) if first_sample_time else 0
    print(f"\n\nCollection complete:")
    print(f"  Duration: {data_duration:.1f}s")
    print(f"  Samples: {sample_count} -> {csv_path}")
    print(f"  Retransmits received: {retransmit_count}")
    total = sample_count + gap_count
    loss_pct = gap_count / total * 100 if total > 0 else 0
    print(f"  Gaps (lost): {gap_count} ({loss_pct:.2f}%)")
    if meta_written:
        with open(meta_path, "a", encoding="utf-8") as f:
            # Final calibration write — ensures all late-arriving cal data is saved
            if cal.has_data:
                f.write(f"\n# Final calibration (complete)\n")
                cal.write_to_file(f)
            f.write(f"\n# Collection summary\n")
            f.write(f"duration_s={data_duration:.3f}\n")
            f.write(f"sample_count={sample_count}\n")
            f.write(f"gap_count={gap_count}\n")
        print(f"  Metadata: {meta_path}")


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
    args = parser.parse_args()

    try:
        import hid  # noqa: F401
    except ImportError:
        print("Error: hidapi is required. Install with: pip install hidapi")
        sys.exit(1)

    collect_hid(args.output, args.duration, args.device)


if __name__ == "__main__":
    main()
