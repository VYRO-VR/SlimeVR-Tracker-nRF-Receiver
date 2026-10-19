#!/usr/bin/env python3
"""Actual FEM backend + raw scan, with timed hardware leaves (no RF proof)."""
from pathlib import Path
import re
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

def source(name):
    return re.sub(r'^#include[^\n]*\n', '', (ROOT / 'src/connection' / name).read_text(), flags=re.M)

fixture = (HERE / 'fem_fixture.c').read_text()
fixture = fixture.replace('/* FEM_HEADER */', source('rssi_scan_fem.h'))
fixture = fixture.replace('/* FEM_SOURCE */', source('rssi_scan_fem.c'))
fixture = fixture.replace('/* SCAN_HEADER */', source('rssi_scan.h').replace('#pragma once', ''))
fixture = fixture.replace('/* SCAN_SOURCE */', source('rssi_scan.c'))
with tempfile.TemporaryDirectory(prefix='rssi-fem-') as tmp:
    path = Path(tmp)
    (path / 'fixture.c').write_text(fixture)
    for fem in (0, 1):
        binary = path / f'test-{fem}'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                        '-fno-pie', '-no-pie', f'-DTEST_FEM={fem}',
                        str(path / 'fixture.c'), '-o', str(binary)], check=True)
        for failure in (['clear', 'disable'] if fem else ['none']):
            subprocess.run([str(binary), failure], check=True)
