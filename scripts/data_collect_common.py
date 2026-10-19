#!/usr/bin/env python3
"""Shared controls and metadata-repair scheduling for raw collectors."""

from __future__ import annotations

import os
import queue
import threading
import time

SLIME_VID = 0x1209
SLIME_PID = 0x7690
DC_USAGE_PAGE = 0xFF00
CONTROL_USAGE_PAGE = 0x01
COLLECTMETA_OPCODE = 223
META_MASK_ALL = 0x3F
META_MASK_BASIC = 1 << 0
META_MASK_ACCEL = 1 << 1
META_MASK_MAG = 1 << 2
META_MASK_GYRO = 1 << 3
META_MASK_TCAL_STATE = 1 << 4
META_MASK_TCAL_POINTS = 1 << 5


class QueuedControlSender:
    """Keep bounded control I/O off the raw-data reader thread."""

    def __init__(self):
        self._pending = queue.Queue(maxsize=16)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        warned = False
        try:
            while not self._stop.is_set():
                try:
                    request = self._pending.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    self._send(*request)
                except Exception as exc:
                    if not warned:
                        print(f"Metadata repair control failed: {exc}")
                        warned = True
        finally:
            self._close_transport()

    def request_metadata(self, tracker_id: int, mask: int, chunk: int) -> bool:
        if self._stop.is_set():
            return False
        try:
            self._pending.put_nowait((tracker_id, mask, chunk))
            return True
        except queue.Full:
            return False

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1)


class SerialControlSender(QueuedControlSender):
    """Send console collectmeta commands over an explicitly selected port."""

    def __init__(self, port: str, baudrate: int = 115200):
        import serial

        self.port = port
        self._serial = serial.Serial(
            port, baudrate=baudrate, timeout=0.2, write_timeout=0.2
        )
        super().__init__()

    def _send(self, tracker_id: int, mask: int, chunk: int) -> None:
        line = f"collectmeta {tracker_id} {mask} {chunk}\n".encode("ascii")
        if self._serial.write(line) != len(line):
            raise OSError("Incomplete metadata request write")

    def _close_transport(self) -> None:
        self._serial.close()


class HidControlSender(QueuedControlSender):
    """Send commands to the receiver's explicitly matched HID endpoint."""

    def __init__(self, path: bytes):
        from hid_cmd import HidCmdClient

        self._client = HidCmdClient(device_path=path)
        super().__init__()

    def _send(self, tracker_id: int, mask: int, chunk: int) -> None:
        status, _ = self._client.command(
            COLLECTMETA_OPCODE, bytes((tracker_id, mask, chunk)), timeout_s=0.25
        )
        if status not in (0, 6):
            raise OSError(f"Receiver rejected metadata request: status={status}")

    def _close_transport(self) -> None:
        self._client.close()


def _serial_for_path(path: str | bytes | None) -> str | None:
    if path is None:
        return None
    try:
        from serial.tools import list_ports

        wanted = os.fspath(path)
        for item in list_ports.comports():
            if item.device == wanted:
                value = getattr(item, "serial_number", None)
                return str(value) if value else None
    except (ImportError, OSError):
        return None
    return None


def auto_hid_control_sender(
    receiver_serial: str | None,
    data_hid_path: bytes | None = None,
    *,
    warn=print,
):
    """Find a control HID endpoint only when its receiver serial matches."""
    if not receiver_serial:
        warn("Metadata repair control unavailable: receiver serial is unknown; use --control-port.")
        return None
    try:
        import hid
    except ImportError:
        warn("Metadata repair control unavailable: hidapi is not installed; use --control-port.")
        return None

    matches = []
    for item in hid.enumerate(SLIME_VID, SLIME_PID):
        serial_number = item.get("serial_number")
        if not serial_number or str(serial_number) != str(receiver_serial):
            continue
        if data_hid_path is not None and item.get("path") == data_hid_path:
            continue
        usage_page = item.get("usage_page")
        # The data endpoint is 0xFF00. Prefer the receiver's standard/control
        # endpoint and reject ambiguous vendor endpoints rather than guessing.
        if usage_page == DC_USAGE_PAGE:
            continue
        if usage_page not in (0x01, CONTROL_USAGE_PAGE, None):
            continue
        matches.append(item)

    if len(matches) != 1:
        if not matches:
            warn(
                f"Metadata repair control unavailable: no HID control endpoint matched receiver serial {receiver_serial!r}."
            )
        else:
            warn(
                f"Metadata repair control unavailable: multiple HID control endpoints matched receiver serial {receiver_serial!r}."
            )
        return None
    try:
        return HidControlSender(matches[0]["path"])
    except Exception as exc:
        warn(f"Metadata repair control unavailable: cannot open matched HID endpoint: {exc}")
        return None


def make_control_sender(
    control_port: str | None = None,
    *,
    receiver_serial: str | None = None,
    data_hid_path: bytes | None = None,
    baudrate: int = 115200,
    warn=print,
):
    """Build explicit serial control or serial-matched HID control."""
    if control_port:
        try:
            return SerialControlSender(control_port, baudrate)
        except Exception as exc:
            warn(f"Metadata repair control unavailable on {control_port}: {exc}")
            return None
    return auto_hid_control_sender(receiver_serial, data_hid_path, warn=warn)


class MetadataRepairScheduler:
    """Rate-limit per-tracker missing-metadata requests after startup grace."""

    def __init__(
        self,
        sender=None,
        *,
        session_start: float | None = None,
        grace_s: float = 5.0,
        retry_s: float = 2.0,
        clock=time.monotonic,
        warn=print,
    ):
        self.sender = sender
        self.session_start = clock() if session_start is None else session_start
        self.grace_s = grace_s
        self.retry_s = retry_s
        self.clock = clock
        self.warn = warn
        self._last_request: dict[int, float] = {}
        self._warned_no_control = False

    def service(self, states, now: float | None = None) -> None:
        now = self.clock() if now is None else now
        if now - self.session_start < self.grace_s:
            return
        for state in states:
            tracker_id = getattr(state, "control_tracker_id", None)
            if tracker_id is None:
                continue
            if state.metadata_complete():
                self._last_request.pop(tracker_id, None)
                continue
            last = self._last_request.get(tracker_id)
            if last is not None and now - last < self.retry_s:
                continue
            request = state.missing_metadata_request()
            if request is None:
                continue
            mask, chunk = request
            self._last_request[tracker_id] = now
            if self.sender is None:
                if not self._warned_no_control:
                    self.warn(
                        "Metadata repair control unavailable; missing metadata will not be requested "
                        "(provide --control-port or a serial-matched receiver HID control endpoint)."
                    )
                    self._warned_no_control = True
                continue
            try:
                accepted = self.sender.request_metadata(tracker_id, mask, chunk)
            except Exception as exc:
                accepted = False
                self.warn(f"Metadata repair request failed for tracker {tracker_id}: {exc}")
            if not accepted:
                self.warn(
                    f"Metadata repair request rejected for tracker {tracker_id} "
                    f"(mask=0x{mask:02x}, chunk={chunk})."
                )

    def close(self) -> None:
        if self.sender is not None:
            self.sender.close()
            self.sender = None


def receiver_serial_for_cdc(port: str | None) -> str | None:
    return _serial_for_path(port)
