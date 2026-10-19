#!/usr/bin/env python3
"""Exercise actual RSSI polling/scan with RADIO events and a 32768-Hz RTC."""
from pathlib import Path
import re
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
source = (ROOT / "src/connection/rssi_scan.c").read_text()
source = re.sub(r"^#include[^\n]*\n", "", source, flags=re.M)
fixture = (HERE / "rssi_fixture.c").read_text().replace("/* ACTUAL_SOURCE */", source)
with tempfile.TemporaryDirectory(prefix="rssi-events-") as tmp:
    path = Path(tmp)
    (path / "fixture.c").write_text(fixture)
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "src/connection"), "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie", str(path / "fixture.c"), "-o", str(path / "test")], check=True)
    subprocess.run([str(path / "test")], check=True)
