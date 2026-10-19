#!/usr/bin/env python3
"""Exercise actual radio lifecycle and private-frame ingress with mocked SDK leaves.

Whole lifecycle/restore/receive functions are retained verbatim. For the large
ACK and event handlers, retain their actual leading private-frame dispatch and
TX completion branches; replace only downstream tracker processing with counted
sentinels. These prove early isolation, not tracker decoding or hardware timing.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
SOURCE = ROOT / "src/connection"


def function(source, name, occurrence=0):
    matches = list(re.finditer(
        rf"^(?:static\s+)?(?:int|void|bool|uint32_t)\s+{name}\s*\([^;]*?\)\s*\{{",
        source, re.M))
    start = matches[occurrence].start()
    opening = source.index("{", matches[occurrence].start())
    # Production functions selected here have balanced braces in comments and
    # strings. Ignore both explicitly so future prose cannot change extraction.
    tokens = re.finditer(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|/\*.*?\*/|//[^\n]*|[{}]',
                         source[opening:], re.S)
    depth = 0
    for token in tokens:
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[start:opening + token.end()]
    raise ValueError(f"Unterminated function {name}")


esb = (SOURCE / "esb.c").read_text()
auto = (SOURCE / "auto_channel.c").read_text()
lifecycle = "\n\n".join(function(esb, name) for name in (
    "esb_radio_quiesce", "esb_initialize_internal", "esb_initialize",
    "esb_initialize_quiet", "esb_home_beacon_send", "esb_radio_operation_stop",
    "radio_operation_begin", "esb_radio_operation_begin", "esb_radio_operation_end",
    "channel_apply"))
ack = function(esb, "esb_ack_handler_cb").split("/* ---- Discovery pairing packets", 1)[0]
ack += "\n downstream_ack++; (void)pipe_id; (void)ack_payload;\n}\n"
event = function(esb, "event_handler").split("/* Discovery has a checksum byte", 1)[0]
event += "\n (void)current_rx_ticks;\n downstream_pose++; downstream_pairing++; downstream_command++;\n}\nbreak;\n}\n default: break;\n}\n}\n"
common = function(auto, "auto_channel_is_beacon")
receive_on = function(auto, "auto_channel_receive")
receive_off = function(auto, "auto_channel_receive", 1)
restore = function(auto, "restore")
fixture = (HERE / "radio_fixture.c").read_text()
fixture = '#include "rssi_scan_fem.h"\n' + fixture
fixture = fixture.replace("/* ACTUAL_COMMON */", common)
fixture = fixture.replace("/* ACTUAL_RECEIVE_ON */", receive_on)
fixture = fixture.replace("/* ACTUAL_RECEIVE_OFF */", receive_off)
fixture = fixture.replace("/* ACTUAL_ACK */", ack)
fixture = fixture.replace("/* ACTUAL_EVENT */", event)
fixture = fixture.replace("/* ACTUAL_LIFECYCLE */", lifecycle)
fixture = fixture.replace("/* ACTUAL_RESTORE */", restore)
with tempfile.TemporaryDirectory(prefix="auto-channel-radio-") as directory:
    directory = Path(directory)
    source = directory / "radio.c"
    source.write_text(fixture)
    for enabled in (0, 1):
        exe = directory / f"radio-{enabled}"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                        "-I", str(SOURCE), f"-DTEST_FEATURE_ON={enabled}", str(source), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
