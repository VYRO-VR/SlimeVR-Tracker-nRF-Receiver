#!/usr/bin/env python
# /// script
# dependencies = [
#   "hidapi",
# ]
# ///
"""
HID control CLI for SlimeNRF receiver (type 254 / ACK 251).

Mirrors constants in src/rcv_hid_cmd.h and ESB_PONG_FLAG_* in src/connection/esb.h.

Examples:
  hid_cmd.py nop
  hid_cmd.py send 0 meow
  hid_cmd.py send all ping
  hid_cmd.py send 1 mag on
  hid_cmd.py send all sens 1.0,1.0,1.0
  hid_cmd.py send 0 sens auto z 5
  hid_cmd.py send all reset zro
  hid_cmd.py send all tcal auto on
  hid_cmd.py tracker-channel 25
  hid_cmd.py --gui
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
from typing import Callable

try:
    import hid
except ImportError:
    print("Error: hidapi not installed. Run: pip install hidapi")
    sys.exit(1)

VID = 0x1209
PID = 0x7690
REPORT_SIZE = 64

RCV_HID_TYPE_CMD_ACK = 251
RCV_HID_TYPE_CMD = 254

RCV_HID_TARGET_ALL = 0xFF

RCV_HID_OP_PAIR = 201
RCV_HID_OP_EXIT_PAIR = 202
RCV_HID_OP_CLEAR = 203
RCV_HID_OP_ADD = 204
RCV_HID_OP_REMOVE = 205
RCV_HID_OP_LIST = 206
RCV_HID_OP_CHANNEL_SET = 207
RCV_HID_OP_CHANNEL_CLEAR = 208
RCV_HID_OP_RSSI_SCAN = 209
RCV_HID_OP_INFO = 210
RCV_HID_OP_UPTIME = 211
RCV_HID_OP_STATS = 212
RCV_HID_OP_RESETSTATS = 213
RCV_HID_OP_COLLECT_START = 214
RCV_HID_OP_COLLECT_STOP = 215
RCV_HID_OP_REBOOT = 216
RCV_HID_OP_DFU = 217
RCV_HID_OP_TRACKER_CH_ALL = 218
RCV_HID_OP_TRACKER_CH_CLR = 219
RCV_HID_OP_NOP = 220

# ESB_PONG_FLAG_* (src/connection/esb.h) — full set except NORMAL / SET_CHANNEL / CLEAR_CHANNEL
# Channel-all uses dongle opcodes 218/219 instead of 0x0A/0x0B.
PONG_FLAG = {
    "shutdown": 0x01,
    "calibrate": 0x02,
    "6-side": 0x03,
    "meow": 0x04,
    "scan": 0x05,
    "mag-clear": 0x06,
    "reboot": 0x07,
    "clear": 0x08,
    "dfu": 0x09,
    "sens-set": 0x0C,
    "sens-reset": 0x0D,
    "reset-zro": 0x0E,
    "reset-acc": 0x0F,
    "reset-bat": 0x10,
    "ping": 0x11,
    "reset-tcal": 0x12,
    "tcal-auto-on": 0x13,
    "tcal-auto-off": 0x14,
    "fusion": 0x15,
    "fusion-reset": 0x15,
    "tcal-boot-on": 0x16,
    "tcal-boot-off": 0x17,
    "mag-cal": 0x18,
    "mag-on": 0x19,
    "mag-off": 0x1A,
    "tcal-on": 0x1B,
    "tcal-off": 0x1C,
    "tdma-on": 0x1D,
    "tdma-off": 0x1E,
    "test-on": 0x1F,
    "test-off": 0x20,
    "dfu-ota": 0x21,
    "collect-on": 0x22,
    "collect-off": 0x23,
    "sens-auto": 0x24,
    "mag-auto-on": 0x25,
    "mag-auto-off": 0x26,
    "ota-query-info": 0x30,
    "ota-abort": 0x31,
    "ota-suppress": 0x32,
    "ota-unsuppress": 0x33,
}

STATUS_NAMES = {
    0: "OK",
    1: "EINVAL",
    2: "ENOSPC",
    3: "EBUSY",
    4: "ENOENT",
    5: "ENOTSUP",
    6: "QUEUED",
    7: "STARTED",
}
RCV_HID_ST_OK = 0
RCV_HID_ST_QUEUED = 6
RCV_HID_ST_STARTED = 7

_seq = 0


def next_seq() -> int:
    global _seq
    _seq = (_seq + 1) & 0xFF
    if _seq == 0:
        _seq = 1
    return _seq


def parse_target(s: str) -> int:
    if s == "all":
        return RCV_HID_TARGET_ALL
    tid = int(s, 0)
    if tid < 0 or tid > 255:
        raise ValueError(f"invalid tracker id: {s}")
    return tid


def scale_sens(v: float) -> int:
    iv = int(round(v * 100.0))
    if iv < -32768 or iv > 32767:
        raise ValueError(f"sens value out of int16 range: {v}")
    return iv


def enumerate_receivers() -> list[dict]:
    devices = hid.enumerate(VID, PID)
    seen: set[bytes] = set()
    unique = []
    for d in devices:
        path = d["path"]
        if path not in seen:
            seen.add(path)
            unique.append(d)
    return unique


class HidCmdClient:
    def __init__(self, device_path: bytes | None = None):
        self.dev = hid.device()
        if device_path:
            self.dev.open_path(device_path)
        else:
            self.dev.open(VID, PID)
        self.dev.set_nonblocking(True)
        print(f"Connected: {self.dev.get_product_string()}")

    def close(self) -> None:
        self.dev.close()

    def _write(self, data: bytes) -> None:
        padded = data.ljust(REPORT_SIZE, b"\x00")
        self.dev.write(b"\x00" + padded)

    def _read(self, timeout_ms: int = 100) -> bytes | None:
        data = self.dev.read(REPORT_SIZE, timeout_ms)
        if data and len(data) > 0:
            return bytes(data)
        return None

    def _wait_ack(self, seq: int, opcode: int, timeout_s: float) -> tuple[int, bytes]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            data = self._read(timeout_ms=50)
            if not data:
                continue
            for offset in range(0, min(len(data), 64), 16):
                sub = data[offset : offset + 16]
                if len(sub) < 4:
                    continue
                if sub[0] == RCV_HID_TYPE_CMD_ACK and sub[1] == seq and sub[2] == opcode:
                    return sub[3], bytes(sub[4:])
        raise TimeoutError(f"No ACK for opcode {opcode} seq {seq}")

    def command(
        self,
        opcode: int,
        args: bytes = b"",
        timeout_s: float = 2.0,
        wait_completion: bool = False,
        completion_timeout_s: float = 35.0,
    ) -> tuple[int, bytes]:
        seq = next_seq()
        pkt = bytearray(REPORT_SIZE)
        pkt[0] = RCV_HID_TYPE_CMD
        pkt[1] = seq
        pkt[2] = opcode
        pkt[3] = 0
        pkt[4 : 4 + len(args)] = args[:12]
        self._write(bytes(pkt))

        status, payload = self._wait_ack(seq, opcode, timeout_s)
        if wait_completion and status == RCV_HID_ST_STARTED:
            print_status(status, payload)
            print("waiting for completion ACK…")
            status, payload = self._wait_ack(seq, opcode, completion_timeout_s)
        return status, payload


def print_status(status: int, payload: bytes = b"") -> None:
    name = STATUS_NAMES.get(status, f"UNKNOWN({status})")
    extra = f" payload={payload[:8].hex()}" if any(payload) else ""
    print(f"ACK status={status} ({name}){extra}")


def build_send(target: str, tokens: list[str]) -> tuple[int, bytes]:
    """Map console-like send tokens → (opcode, args)."""
    if not tokens:
        raise ValueError("missing remote command")

    tid = parse_target(target)
    cmd = tokens[0]
    rest = tokens[1:]

    # Compound: mag …
    if cmd == "mag":
        if not rest:
            raise ValueError("Usage: send <id|all> mag <on|off|clear|cal|auto on|auto off>")
        sub = rest[0]
        if sub == "on":
            return PONG_FLAG["mag-on"], bytes([tid])
        if sub == "off":
            return PONG_FLAG["mag-off"], bytes([tid])
        if sub == "clear":
            return PONG_FLAG["mag-clear"], bytes([tid])
        if sub in ("cal", "calibrate"):
            return PONG_FLAG["mag-cal"], bytes([tid])
        if sub in ("auto", "online"):
            if len(rest) < 2 or rest[1] not in ("on", "off"):
                raise ValueError("Usage: send <id|all> mag auto <on|off>")
            key = "mag-auto-on" if rest[1] == "on" else "mag-auto-off"
            return PONG_FLAG[key], bytes([tid])
        raise ValueError(f"Unknown mag subcommand: {sub}")

    # Compound: sens …
    if cmd == "sens":
        if not rest:
            raise ValueError(
                "Usage: send <id|all> sens <x,y,z> | reset | auto <x|y|z> [revolutions]"
            )
        if rest[0] == "reset":
            return PONG_FLAG["sens-reset"], bytes([tid])
        if rest[0] == "auto":
            if len(rest) < 2 or rest[1] not in ("x", "y", "z"):
                raise ValueError("Usage: send <id|all> sens auto <x|y|z> [revolutions]")
            axis = {"x": 0, "y": 1, "z": 2}[rest[1]]
            rev = int(rest[2]) if len(rest) >= 3 else 0
            args = bytes([tid, axis]) + struct.pack("<H", rev)
            return PONG_FLAG["sens-auto"], args
        # x,y,z
        parts = rest[0].split(",")
        if len(parts) != 3:
            raise ValueError("Usage: send <id|all> sens <x>,<y>,<z>")
        xi, yi, zi = (scale_sens(float(p)) for p in parts)
        args = bytes([tid]) + struct.pack("<hhh", xi, yi, zi)
        return PONG_FLAG["sens-set"], args

    # Compound: reset …
    if cmd == "reset":
        if not rest:
            raise ValueError("Usage: send <id|all> reset <zro|acc|bat|mag|tcal|fusion>")
        mapping = {
            "zro": "reset-zro",
            "acc": "reset-acc",
            "bat": "reset-bat",
            "mag": "mag-clear",
            "tcal": "reset-tcal",
            "fusion": "fusion-reset",
        }
        if rest[0] not in mapping:
            raise ValueError(f"Unknown reset target: {rest[0]}")
        return PONG_FLAG[mapping[rest[0]]], bytes([tid])

    # Compound: tcal …
    if cmd == "tcal":
        if not rest:
            raise ValueError(
                "Usage: send <id|all> tcal <on|off|reset|auto on|auto off|boot on|boot off>"
            )
        sub = rest[0]
        if sub == "on":
            return PONG_FLAG["tcal-on"], bytes([tid])
        if sub == "off":
            return PONG_FLAG["tcal-off"], bytes([tid])
        if sub == "reset":
            return PONG_FLAG["reset-tcal"], bytes([tid])
        if sub == "auto":
            if len(rest) < 2 or rest[1] not in ("on", "off"):
                raise ValueError("Usage: send <id|all> tcal auto <on|off>")
            key = "tcal-auto-on" if rest[1] == "on" else "tcal-auto-off"
            return PONG_FLAG[key], bytes([tid])
        if sub == "boot":
            if len(rest) < 2 or rest[1] not in ("on", "off"):
                raise ValueError("Usage: send <id|all> tcal boot <on|off>")
            key = "tcal-boot-on" if rest[1] == "on" else "tcal-boot-off"
            return PONG_FLAG[key], bytes([tid])
        raise ValueError(f"Unknown tcal subcommand: {sub}")

    # Compound: tdma / test
    if cmd == "tdma":
        if not rest or rest[0] not in ("on", "off"):
            raise ValueError("Usage: send <id|all> tdma <on|off>")
        return PONG_FLAG["tdma-on" if rest[0] == "on" else "tdma-off"], bytes([tid])

    if cmd == "test":
        if not rest or rest[0] not in ("on", "off"):
            raise ValueError("Usage: send <id|all> test <on|off>")
        return PONG_FLAG["test-on" if rest[0] == "on" else "test-off"], bytes([tid])

    # dfu [ota]
    if cmd == "dfu":
        if rest and rest[0] == "ota":
            return PONG_FLAG["dfu-ota"], bytes([tid])
        if rest:
            raise ValueError("Usage: send <id|all> dfu [ota]")
        return PONG_FLAG["dfu"], bytes([tid])

    # Tracker RF channel — dongle opcodes (all only)
    if cmd == "channel":
        if tid != RCV_HID_TARGET_ALL:
            raise ValueError("channel via HID requires target 'all' (use tracker-channel)")
        if not rest:
            raise ValueError("Usage: send all channel <1-100>")
        ch = int(rest[0])
        if ch < 1 or ch > 100:
            raise ValueError("channel must be 1-100")
        return RCV_HID_OP_TRACKER_CH_ALL, bytes([ch])

    if cmd == "clearchannel":
        if tid != RCV_HID_TARGET_ALL:
            raise ValueError("clearchannel via HID requires target 'all'")
        return RCV_HID_OP_TRACKER_CH_CLR, b""

    # Raw opcode: send <id|all> raw 0x1B
    if cmd == "raw":
        if not rest:
            raise ValueError("Usage: send <id|all> raw <opcode>")
        opcode = int(rest[0], 0)
        return opcode, bytes([tid])

    # Flat aliases (mag-on, tcal-auto-on, …) and simple console names
    if cmd in PONG_FLAG:
        return PONG_FLAG[cmd], bytes([tid])

    # fusion as alias already in map; 6-side etc.
    raise ValueError(
        f"Unknown remote command: {cmd}\n"
        "See: shutdown calibrate 6-side meow scan mag reboot clear dfu fusion sens "
        "reset ping tcal tdma test collect-on/off ota-* raw <op>"
    )


def run_gui() -> int:
    try:
        import tkinter as tk
        from tkinter import messagebox, scrolledtext, ttk
    except ImportError as exc:
        print(f"tkinter unavailable: {exc}")
        print("WSL: sudo apt install python3-tk; need DISPLAY/WSLg")
        return 1

    class HidCmdGui(tk.Tk):
        def __init__(self) -> None:
            super().__init__()
            self.title("SlimeNRF HID Cmd")
            self.geometry("720x520")
            self.client: HidCmdClient | None = None
            self._busy = False
            self._paths: list[bytes] = []
            self._build()
            self.refresh_devices()
            self.protocol("WM_DELETE_WINDOW", self.on_close)

        def _build(self) -> None:
            top = ttk.Frame(self, padding=8)
            top.pack(fill=tk.X)

            ttk.Label(top, text="Device").pack(side=tk.LEFT)
            self.dev_var = tk.StringVar()
            self.dev_combo = ttk.Combobox(
                top, textvariable=self.dev_var, width=48, state="readonly"
            )
            self.dev_combo.pack(side=tk.LEFT, padx=6)
            ttk.Button(top, text="Refresh", command=self.refresh_devices).pack(side=tk.LEFT)
            self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
            self.conn_btn.pack(side=tk.LEFT, padx=6)

            row = ttk.Frame(self, padding=(8, 0))
            row.pack(fill=tk.X)
            ttk.Label(row, text="Target").pack(side=tk.LEFT)
            self.target_var = tk.StringVar(value="all")
            ttk.Entry(row, textvariable=self.target_var, width=8).pack(side=tk.LEFT, padx=4)
            ttk.Label(row, text="(id or all)").pack(side=tk.LEFT)

            quick = ttk.LabelFrame(self, text="Quick send", padding=8)
            quick.pack(fill=tk.X, padx=8, pady=6)
            for i, (label, tokens) in enumerate(
                [
                    ("meow", ["meow"]),
                    ("ping", ["ping"]),
                    ("calibrate", ["calibrate"]),
                    ("reboot", ["reboot"]),
                    ("mag on", ["mag", "on"]),
                    ("mag off", ["mag", "off"]),
                    ("tcal on", ["tcal", "on"]),
                    ("tdma on", ["tdma", "on"]),
                    ("test off", ["test", "off"]),
                ]
            ):
                ttk.Button(
                    quick,
                    text=label,
                    command=lambda t=tokens: self.run_send(t),
                ).grid(row=i // 5, column=i % 5, padx=3, pady=3, sticky="ew")

            dongle = ttk.LabelFrame(self, text="Dongle", padding=8)
            dongle.pack(fill=tk.X, padx=8, pady=4)
            for i, (label, fn) in enumerate(
                [
                    ("nop", lambda: self.run_op(RCV_HID_OP_NOP)),
                    ("info", lambda: self.run_op(RCV_HID_OP_INFO)),
                    ("list", lambda: self.run_op(RCV_HID_OP_LIST)),
                    ("pair", lambda: self.run_op(RCV_HID_OP_PAIR, bytes([0]))),
                    ("exit pair", lambda: self.run_op(RCV_HID_OP_EXIT_PAIR)),
                    ("clear", lambda: self.run_op(RCV_HID_OP_CLEAR)),
                    ("rssi_scan", lambda: self.run_op(RCV_HID_OP_RSSI_SCAN)),
                    ("resetstats", lambda: self.run_op(RCV_HID_OP_RESETSTATS)),
                ]
            ):
                ttk.Button(dongle, text=label, command=fn).grid(
                    row=0, column=i, padx=3, pady=2, sticky="ew"
                )

            ch = ttk.Frame(dongle)
            ch.grid(row=1, column=0, columnspan=8, sticky="w", pady=(8, 0))
            ttk.Label(ch, text="Tracker ch").pack(side=tk.LEFT)
            self.ch_var = tk.IntVar(value=25)
            ttk.Spinbox(ch, from_=1, to=100, textvariable=self.ch_var, width=5).pack(
                side=tk.LEFT, padx=4
            )
            ttk.Button(ch, text="Set all", command=self.run_tracker_channel).pack(
                side=tk.LEFT
            )
            ttk.Button(ch, text="Clear all", command=self.run_tracker_clearchannel).pack(
                side=tk.LEFT, padx=4
            )

            send_row = ttk.Frame(self, padding=8)
            send_row.pack(fill=tk.X)
            ttk.Label(send_row, text="send").pack(side=tk.LEFT)
            self.send_var = tk.StringVar(value="meow")
            entry = ttk.Entry(send_row, textvariable=self.send_var)
            entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
            entry.bind("<Return>", lambda _e: self.run_send_line())
            ttk.Button(send_row, text="Send", command=self.run_send_line).pack(side=tk.LEFT)

            self.log = scrolledtext.ScrolledText(
                self, height=16, state=tk.DISABLED, wrap=tk.WORD
            )
            self.log.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        def log_line(self, msg: str) -> None:
            def _append() -> None:
                self.log.configure(state=tk.NORMAL)
                self.log.insert(tk.END, msg + "\n")
                self.log.see(tk.END)
                self.log.configure(state=tk.DISABLED)

            self.after(0, _append)

        def refresh_devices(self) -> None:
            devices = enumerate_receivers()
            labels = []
            self._paths = []
            for i, d in enumerate(devices):
                labels.append(
                    f"[{i}] {d.get('product_string') or 'receiver'} "
                    f"serial={d.get('serial_number') or '-'}"
                )
                self._paths.append(d["path"])
            self.dev_combo["values"] = labels
            if labels:
                self.dev_combo.current(0)
            else:
                self.dev_var.set("")
                self.log_line("No receivers found.")

        def toggle_connect(self) -> None:
            if self.client:
                self.client.close()
                self.client = None
                self.conn_btn.configure(text="Connect")
                self.log_line("Disconnected.")
                return
            idx = self.dev_combo.current()
            if idx < 0 or idx >= len(self._paths):
                messagebox.showerror("HID", "No device selected")
                return
            try:
                self.client = HidCmdClient(self._paths[idx])
                self.conn_btn.configure(text="Disconnect")
                self.log_line("Connected.")
            except Exception as exc:
                messagebox.showerror("HID", str(exc))

        def _ensure_client(self) -> HidCmdClient | None:
            if self.client:
                return self.client
            messagebox.showerror("HID", "Connect first")
            return None

        def _bg(self, work: Callable[[], None]) -> None:
            if self._busy:
                self.log_line("Busy…")
                return
            self._busy = True

            def runner() -> None:
                try:
                    work()
                except Exception as exc:
                    self.log_line(f"Error: {exc}")
                finally:
                    self._busy = False

            threading.Thread(target=runner, daemon=True).start()

        def run_op(
            self, opcode: int, args: bytes = b"", wait_completion: bool = False
        ) -> None:
            client = self._ensure_client()
            if not client:
                return

            def work() -> None:
                st, pl = client.command(opcode, args, wait_completion=wait_completion)
                name = STATUS_NAMES.get(st, f"?({st})")
                extra = f" payload={pl[:8].hex()}" if any(pl) else ""
                self.log_line(f"ACK opcode={opcode} status={st} ({name}){extra}")

            self._bg(work)

        def run_send(self, tokens: list[str]) -> None:
            client = self._ensure_client()
            if not client:
                return
            target = self.target_var.get().strip() or "all"

            def work() -> None:
                opcode, payload = build_send(target, tokens)
                wait_done = (
                    opcode <= 200
                    or opcode in (RCV_HID_OP_TRACKER_CH_ALL, RCV_HID_OP_TRACKER_CH_CLR)
                )
                st, pl = client.command(
                    opcode, payload, wait_completion=wait_done, completion_timeout_s=15.0
                )
                name = STATUS_NAMES.get(st, f"?({st})")
                self.log_line(
                    f"send {target} {' '.join(tokens)} → ACK opcode=0x{opcode:02X} "
                    f"status={st} ({name})"
                )

            self._bg(work)

        def run_send_line(self) -> None:
            tokens = self.send_var.get().strip().split()
            if not tokens:
                return
            self.run_send(tokens)

        def run_tracker_channel(self) -> None:
            ch = int(self.ch_var.get())
            self.run_op(RCV_HID_OP_TRACKER_CH_ALL, bytes([ch]), wait_completion=True)

        def run_tracker_clearchannel(self) -> None:
            self.run_op(RCV_HID_OP_TRACKER_CH_CLR, wait_completion=True)

        def on_close(self) -> None:
            if self.client:
                self.client.close()
                self.client = None
            self.destroy()

    try:
        app = HidCmdGui()
    except tk.TclError as exc:
        print(f"tkinter unavailable: {exc}")
        print("WSL: sudo apt install python3-tk; need DISPLAY/WSLg")
        return 1
    app.mainloop()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SlimeNRF receiver HID control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "send examples:\n"
            "  send 0 meow\n"
            "  send all ping\n"
            "  send 1 mag on\n"
            "  send all mag auto off\n"
            "  send 0 sens 1.0,1.0,1.0\n"
            "  send 0 sens auto z 5\n"
            "  send all sens reset\n"
            "  send 1 reset zro\n"
            "  send all tcal auto on\n"
            "  send all tdma on\n"
            "  send all test off\n"
            "  send all dfu ota\n"
            "  send all channel 25\n"
            "  send 0 raw 0x04\n"
            "  --gui\n"
        ),
    )
    parser.add_argument("--device", type=int, default=None, help="Receiver index")
    parser.add_argument("--gui", action="store_true", help="Open tkinter test UI")
    sub = parser.add_subparsers(dest="cmd", required=False)

    sub.add_parser("nop")
    sub.add_parser("info")
    sub.add_parser("uptime")
    sub.add_parser("list")
    sub.add_parser("exit")
    sub.add_parser("clear")
    sub.add_parser("remove")
    sub.add_parser("clearchannel")
    sub.add_parser("rssi_scan")
    sub.add_parser("resetstats")
    sub.add_parser("reboot")

    p_pair = sub.add_parser("pair")
    p_pair.add_argument("count", nargs="?", type=int, default=0)

    p_add = sub.add_parser("add")
    p_add.add_argument("addr", help="12 hex digits MAC")

    p_ch = sub.add_parser("channel")
    p_ch.add_argument("channel", type=int)

    p_stats = sub.add_parser("stats")
    p_stats.add_argument("seconds", nargs="?", type=int, default=0)

    p_collect = sub.add_parser("collect")
    p_collect.add_argument("target", help="tracker id or 'off'")

    p_dfu = sub.add_parser("dfu")
    p_dfu.add_argument("--ota", action="store_true")

    p_tch = sub.add_parser("tracker-channel")
    p_tch.add_argument("channel", type=int)

    sub.add_parser("tracker-clearchannel")

    p_send = sub.add_parser("send")
    p_send.add_argument("target", help="tracker id or 'all'")
    p_send.add_argument(
        "remote",
        nargs=argparse.REMAINDER,
        help="remote command tokens (meow | mag on | sens 1,1,1 | …)",
    )

    p_flags = sub.add_parser("flags", help="List all tracker PONG flag names")

    args = parser.parse_args()

    if args.gui:
        return run_gui()

    if not args.cmd:
        parser.error("command required (or pass --gui)")

    if args.cmd == "flags":
        for name, op in sorted(PONG_FLAG.items(), key=lambda kv: (kv[1], kv[0])):
            print(f"  0x{op:02X}  {name}")
        print("  (channel/clearchannel → dongle ops 218/219 via send all channel …)")
        return 0

    devices = enumerate_receivers()
    if not devices:
        print(f"No receivers (VID={VID:#06x} PID={PID:#06x})")
        return 1
    if args.device is not None:
        path = devices[args.device]["path"]
    elif len(devices) == 1:
        path = devices[0]["path"]
    else:
        for i, d in enumerate(devices):
            print(f"  [{i}] {d.get('product_string')} serial={d.get('serial_number')}")
        print("Pass --device N")
        return 1

    client = HidCmdClient(path)
    try:
        if args.cmd == "nop":
            st, pl = client.command(RCV_HID_OP_NOP)
        elif args.cmd == "info":
            st, pl = client.command(RCV_HID_OP_INFO)
        elif args.cmd == "uptime":
            st, pl = client.command(RCV_HID_OP_UPTIME)
        elif args.cmd == "list":
            st, pl = client.command(RCV_HID_OP_LIST)
        elif args.cmd == "exit":
            st, pl = client.command(RCV_HID_OP_EXIT_PAIR)
        elif args.cmd == "clear":
            st, pl = client.command(RCV_HID_OP_CLEAR)
        elif args.cmd == "remove":
            st, pl = client.command(RCV_HID_OP_REMOVE)
        elif args.cmd == "clearchannel":
            st, pl = client.command(RCV_HID_OP_CHANNEL_CLEAR)
        elif args.cmd == "rssi_scan":
            st, pl = client.command(RCV_HID_OP_RSSI_SCAN)
        elif args.cmd == "resetstats":
            st, pl = client.command(RCV_HID_OP_RESETSTATS)
        elif args.cmd == "reboot":
            st, pl = client.command(RCV_HID_OP_REBOOT)
        elif args.cmd == "pair":
            st, pl = client.command(RCV_HID_OP_PAIR, bytes([args.count & 0xFF]))
        elif args.cmd == "add":
            addr = int(args.addr, 16)
            st, pl = client.command(RCV_HID_OP_ADD, addr.to_bytes(6, "little"))
        elif args.cmd == "channel":
            st, pl = client.command(RCV_HID_OP_CHANNEL_SET, bytes([args.channel]))
        elif args.cmd == "stats":
            st, pl = client.command(RCV_HID_OP_STATS, struct.pack("<I", args.seconds))
        elif args.cmd == "collect":
            if args.target == "off":
                st, pl = client.command(RCV_HID_OP_COLLECT_STOP)
            else:
                st, pl = client.command(RCV_HID_OP_COLLECT_START, bytes([int(args.target)]))
        elif args.cmd == "dfu":
            st, pl = client.command(RCV_HID_OP_DFU, bytes([1 if args.ota else 0]))
        elif args.cmd == "tracker-channel":
            st, pl = client.command(
                RCV_HID_OP_TRACKER_CH_ALL, bytes([args.channel]), wait_completion=True
            )
        elif args.cmd == "tracker-clearchannel":
            st, pl = client.command(RCV_HID_OP_TRACKER_CH_CLR, wait_completion=True)
        elif args.cmd == "send":
            tokens = list(args.remote)
            # argparse REMAINDER may keep a leading '--'
            if tokens and tokens[0] == "--":
                tokens = tokens[1:]
            opcode, payload = build_send(args.target, tokens)
            wait_done = (
                opcode <= 200
                or opcode in (RCV_HID_OP_TRACKER_CH_ALL, RCV_HID_OP_TRACKER_CH_CLR)
            )
            st, pl = client.command(
                opcode, payload, wait_completion=wait_done, completion_timeout_s=15.0
            )
        else:
            print("unknown command")
            return 1
        print_status(st, pl)
        return 0 if st in (RCV_HID_ST_OK, RCV_HID_ST_QUEUED, RCV_HID_ST_STARTED) else 1
    except Exception as exc:
        print(f"Error: {exc}")
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
