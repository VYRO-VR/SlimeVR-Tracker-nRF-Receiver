"""Exercise real CLI, HID pump, event decoder and JSONL writer without USB."""
from collections import deque
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
from unittest.mock import patch


def power_records():
    records = []
    for phase in range(1, 8):
        record = bytearray(16)
        record[:5] = bytes((251, 0, 225, 7, 2))
        record[5:9] = (123).to_bytes(4, 'little')
        record[9:11] = phase.to_bytes(2, 'little')
        record[13:] = bytes((0x30, phase, 2 if phase == 6 else 0))
        records.append(bytes(record))
    return records


class Device:
    """USB leaf: ACK event controls and deliver supplied wire notifications."""

    def __init__(self, records):
        self.pending = deque()
        self.records = list(records)
        self.controls = []
        self.closed = False
        self.opened = False

    def open_path(self, path):
        self.opened = True

    def set_nonblocking(self, value):
        pass

    def get_product_string(self):
        return 'test receiver'

    def write(self, report):
        _, packet_type, seq, opcode, _, version, action, target, mask = report[:9]
        assert packet_type == 254 and opcode == 224 and version == 1
        self.controls.append((action, target, mask))
        payload = bytes((1, 1, 3, 10, 0, 0, 0, 0, 0, 0, target,
                         1 | (mask << 1) if action in (1, 2) else 0))
        self.pending.append(bytes((251, seq, opcode, 0)) + payload)
        if action == 1:
            self.pending.extend(self.records)

    def read(self, size, timeout):
        if self.pending:
            return self.pending.popleft()
        raise KeyboardInterrupt

    def close(self):
        self.closed = True


def invoke(client, arguments, device, file_opener=None):
    stdout, stderr = io.StringIO(), io.StringIO()
    with ExitStack() as stack:
        stack.enter_context(patch.object(sys, 'argv', ['hid_cmd.py', *arguments]))
        stack.enter_context(patch.object(client.hid, 'enumerate',
                                        return_value=[{'path': b'fake', 'usage_page': 1}],
                                        create=True))
        stack.enter_context(patch.object(client.hid, 'device', return_value=device, create=True))
        if file_opener is not None:
            stack.enter_context(patch.object(client, 'open', file_opener, create=True))
        stack.enter_context(redirect_stdout(stdout))
        stack.enter_context(redirect_stderr(stderr))
        try:
            result = client.main()
        except SystemExit as exc:
            result = exc.code
    return result, stdout.getvalue(), stderr.getvalue()


def logging_cases(client):
    previous_cwd = Path.cwd()
    with tempfile.TemporaryDirectory(prefix='receiver-jsonl-') as directory:
        try:
            os.chdir(directory)
            records = power_records()
            device = Device(records)
            result, output, errors = invoke(client, ['events-watch'], device)
            assert result == 130 and device.closed
            assert device.controls == [(0, 255, 0), (1, 255, 31), (3, 255, 0)]
            logs = list(Path('.').glob('tracker-events-*.jsonl'))
            assert len(logs) == 1 and logs[0].read_text() == output
            events = [json.loads(line) for line in output.splitlines()]
            assert [e['phase'] for e in events] == [
                'WILL_WOM', 'WILL_SHUTDOWN', 'BOOT', 'WAKE', 'WILL_REBOOT',
                'WOM_CANCELLED', 'WATCHDOG_RESET']
            assert [e['detail'] for e in events] == ['UNKNOWN', 0, 0, 0, 0, 'WOM_FORCED', 0]
            assert all(json.dumps(e, separators=(',', ':')) == line
                       for e, line in zip(events, output.splitlines()))
            assert errors and all(e['kind'] == 'POWER' for e in events)

            capture = Path('existing.jsonl')
            prefix = '{"previous":true}\n'
            capture.write_text(prefix)
            device = Device(records)
            result, output, _ = invoke(client, ['events-watch', '2', '--kinds', 'power',
                                                '--out', str(capture)], device)
            assert result == 130 and capture.read_text() == prefix + output
            assert device.closed and device.controls == [(0, 2, 0), (1, 2, 8), (3, 2, 0)]

            files = set(Path('.').iterdir())
            device = Device(records)
            result, output, _ = invoke(client, ['events-watch', '--no-log'], device)
            assert result == 130 and device.closed
            assert [json.loads(line)['phase'] for line in output.splitlines()] == [e['phase'] for e in events]
            assert set(Path('.').iterdir()) == files
            calibration_watch_case(client)
            assert set(Path('.').iterdir()) == files

            for arguments in (
                ['--out', 'invalid.jsonl', '--no-log'],
                ['16', '--out', 'invalid.jsonl'],
                ['--kinds', 'invalid', '--out', 'invalid.jsonl'],
            ):
                device = Device(records)
                result, output, errors = invoke(client, ['events-watch', *arguments], device)
                assert result == 2 and not output and errors
                assert not device.opened and not Path('invalid.jsonl').exists()

            device = Device(records)
            result, output, errors = invoke(client, ['events-watch', '--out', 'missing/log.jsonl'], device)
            assert result == 1 and not output and errors
            assert device.closed and not device.controls and not Path('missing').exists()

            writer_drain_case(client)
            writer_failure_cases(client, records)
        finally:
            os.chdir(previous_cwd)
    print('PASS python-jsonl-capture')


def calibration_watch_case(client):
    class CalibrationDevice(Device):
        def write(self, report):
            if report[3] == 224:
                return super().write(report)
            self.pending.append(bytes((251, report[2], report[3], 0)) + bytes(12))
            for event in (1, 0x14):  # USER ACCEPTED then successful END.
                record = bytearray(16)
                record[:5] = bytes((251, 0, 225, event, 2))
                record[11:13] = (42).to_bytes(2, 'little')
                record[13] = 1
                self.pending.append(bytes(record))

    device = CalibrationDevice([])
    result, output, _ = invoke(client, ['send', '--watch-calibration', '2', 'calibrate'], device)
    assert result == 0 and device.closed
    assert [json.loads(line)['event'] for line in output.splitlines()] == ['ACCEPTED', 'END']
    assert device.controls == [(0, 2, 0), (1, 2, 1), (3, 2, 0)]


def writer_drain_case(client):
    entered, release, closing, finished = (threading.Event() for _ in range(4))
    contents = []
    failures = []

    class BlockedFile:
        closed = False

        def write(self, line):
            entered.set()
            if not release.wait(10):
                raise OSError('test writer release timed out')
            if self.closed:
                failures.append('write after close')
            contents.append(line)

        def flush(self):
            pass

        def close(self):
            if not release.is_set():
                failures.append('closed while write blocked')
            self.closed = True

    destination = BlockedFile()
    with patch.object(client, 'open', return_value=destination, create=True):
        writer = client.JsonlEventWriter('blocked.jsonl')
    lines = [json.dumps({'sequence': n}) for n in range(100)]
    for line in lines:
        writer.write(line)

    def close():
        closing.set()
        try:
            writer.close()
        finally:
            finished.set()

    closer = threading.Thread(target=close)
    closer.start()
    try:
        assert entered.wait(2)
        assert closing.wait(2)
        # Hold I/O past the reference's five-second close timeout.
        assert not finished.wait(5.2)
        assert not destination.closed
    finally:
        release.set()
        closer.join(2)
        writer.close()
    assert finished.is_set() and destination.closed and not failures
    assert contents == [line + '\n' for line in lines]


def writer_failure_cases(client, records):
    for failure in ('write', 'flush', 'close'):
        class FailingFile:
            closed = False
            writes = 0

            def write(self, line):
                self.writes += 1
                if failure == 'write':
                    raise OSError('injected disk failure')

            def flush(self):
                if failure == 'flush':
                    raise OSError('injected disk failure')

            def close(self):
                self.closed = True
                if failure == 'close':
                    raise OSError('injected disk failure')

        destination = FailingFile()
        device = Device(records)
        result, output, errors = invoke(client, ['events-watch', '--out', 'failure.jsonl'],
                                       device, lambda *args, **kwargs: destination)
        assert result == 130 and device.closed and destination.closed
        assert device.controls[-1][0] == 3
        assert [json.loads(line)['phase'] for line in output.splitlines()] == [
            'WILL_WOM', 'WILL_SHUTDOWN', 'BOOT', 'WAKE', 'WILL_REBOOT',
            'WOM_CANCELLED', 'WATCHDOG_RESET']
        assert errors.count('injected disk failure') == 1
        if failure in ('write', 'flush'):
            assert destination.writes == 1


if __name__ == '__main__':
    from run import load_client
    logging_cases(load_client(Path(__file__).resolve().parents[3]))
