#!/usr/bin/env python3
"""Compile the complete production roster module with an atomic NVS leaf model."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[3])
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix='receiver-roster-') as directory:
    root = Path(directory)
    log = root / 'zephyr/logging/log.h'
    log.parent.mkdir(parents=True)
    log.write_text('/* No logging is used by the roster module. */\n')
    executable = root / 'roster'
    subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-g',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        '-I', str(root), '-I', str(args.source_root / 'src'),
        str(args.source_root / 'src/connection/tracker_roster.c'),
        str(Path(__file__).with_name('fixture.c')), '-o', str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True, timeout=20)
