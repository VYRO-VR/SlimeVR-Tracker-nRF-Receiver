#!/usr/bin/env python3
"""Compile production loss-gated policy and radio orchestration with time leaves."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
SOURCE = ROOT / "src/connection"
with tempfile.TemporaryDirectory(prefix="auto-channel-") as tmp:
    tmp = Path(tmp)
    production = (SOURCE / "auto_channel.c").read_text()
    production = re.sub(r"^#include[^\n]*\n", "", production, flags=re.M)
    runtime = (HERE / "runtime_fixture.c").read_text().replace("/* ACTUAL_SOURCE */", production)
    header = (SOURCE / "esb.h").read_text()
    snapshot_types = header.split("#define ESB_AUTO_LOSS_TRACKERS", 1)[1].split(
        "void esb_get_auto_loss_snapshot", 1
    )[0]
    runtime = runtime.replace("/* LOSS_SNAPSHOT_TYPES */", "#define ESB_AUTO_LOSS_TRACKERS" + snapshot_types)
    rssi = (SOURCE / "rssi_scan.c").read_text().split("static uint8_t get_current_effective_channel", 1)[0]
    rssi = re.sub(r"^#include[^\n]*\n", "", rssi, flags=re.M)
    # Manual scanner has its own actual-source fixture; keep only auto timing.
    prefix, manual = rssi.split("static int rssi_scan_channel_measure_min_n", 1)
    rssi = prefix + "int rssi_scan_measure_bounded" + manual.split("int rssi_scan_measure_bounded", 1)[1]
    runtime = runtime.replace("/* RSSI_SOURCE */", rssi)
    (tmp / "runtime.c").write_text(runtime)
    for name, src in (("policy", HERE / "test_policy.c"), ("runtime", tmp / "runtime.c")):
        exe = tmp / name
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(SOURCE), str(src), str(SOURCE / "auto_channel_policy.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        if name == "runtime":
            # New process reloads production static initialization, equivalent
            # to reboot even though the previous process explicitly opted in.
            subprocess.run([str(exe), "--boot-only"], check=True)
