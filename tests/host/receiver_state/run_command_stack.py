#!/usr/bin/env python3
"""Replay the linked receiver roster commands through real NVS and flash.

    uv run --with unicorn --with pyelftools run_command_stack.py PATH/zephyr.elf

Delivers a real `remove`/`clear` command to the linked console thread and to the
linked HID output-report handler. The executed chain is the firmware's own:
message/epoch checks, command dispatch, `esb_pop_pair`/`esb_clear`,
`tracker_roster_save`, `sys_write_checked`, NVS and the nRF flash driver. NVS is
mounted by the linked `sys_nvs_init` on a real pre-filled sector and the roster
is published by the linked `tracker_store_restore`; neither is replaced.

This guards the synchronous persistence transaction against the thread stack
budget it runs on: adding or clearing trackers commits flash on the calling
thread, so a caller that cannot hold the NVS/flash frames reboots before the
commit and leaves the previous roster in storage.

Passing cases execute `remove` after `remove` and `clear`, so a stack fault or a
skipped persistence step is a failure. Measured depth is a LOWER BOUND: modeled
leaves, interrupts, preemption, TLS placement and hardware are excluded, and the
linked budget is the stack object minus the architecture-reserved guard bytes.
Separately require the Cortex-M basic exception frame. This is not hardware
stack-safety proof.
"""

import argparse
import json
from pathlib import Path
import re
import struct
import sys
from collections import Counter

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP,
)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent / 'tdma_loss'))
from run_stack import load_image, SCRATCH_BASE, SCRATCH_SIZE  # noqa: E402
from run_startup_stack import fixture, PAGE_SIZE, ROSTER_ID  # noqa: E402

INSTRUCTION_LIMIT = 4_000_000
EXCEPTION_RESERVE = 32
STOP = SCRATCH_BASE + 0x100
READBACK = SCRATCH_BASE + 0x300
REPORT = SCRATCH_BASE + 0x400
MESSAGE = SCRATCH_BASE + 0x500
INIT_TOP = SCRATCH_BASE + SCRATCH_SIZE
SEEDED = [0x123456780001, 0x123456780002, 0x123456780003]
# Deleted filler ATEs (len == 0) consume ATE space without live data, so the
# sector is left with too few free bytes for one more roster record and the
# command's own write must run the real NVS garbage collection and erase.
GC_FILLER_ENTRIES = 488
CASES = [('console', 'remove', 0), ('console', 'clear', 0),
         ('console', 'remove', GC_FILLER_ENTRIES), ('console', 'clear', GC_FILLER_ENTRIES),
         ('hid', 'remove', 0), ('hid', 'clear', 0)]


def structure_layout(path, name):
    """Members and byte size of a named struct, from this ELF's own DWARF."""
    with path.open('rb') as stream:
        for unit in ELFFile(stream).get_dwarf_info().iter_CUs():
            for die in unit.iter_DIEs():
                attr = die.attributes.get('DW_AT_name')
                if (die.tag != 'DW_TAG_structure_type' or attr is None
                        or attr.value != name.encode()):
                    continue
                size = die.attributes.get('DW_AT_byte_size')
                members = {}
                for member in die.iter_children():
                    member_name = member.attributes.get('DW_AT_name')
                    offset = member.attributes.get('DW_AT_data_member_location')
                    if member_name and offset and isinstance(offset.value, int):
                        members[member_name.value.decode()] = offset.value
                if members and size is not None:
                    return members, size.value
    raise ValueError(f'Missing DWARF layout for {name}')


def hid_opcode(header, name):
    """Read a receiver HID opcode so the replay cannot drift from the header."""
    match = re.search(rf'^#define\s+{name}\s+(0x[0-9a-fA-F]+|\d+)\s*$', header.read_text(), re.M)
    if not match:
        raise ValueError(f'Missing {name} in rcv_hid_cmd.h')
    return int(match.group(1), 0)


def roster_record(count, addresses):
    record = bytes([1, count]) + b''.join(addr.to_bytes(6, 'little') for addr in addresses)
    return record + bytes(6 * (16 - count))


def replay(path, image, target, command, fillers):
    symbols, functions, sections, offsets = image
    geometry, _ = structure_layout(path, 'flash_pages_info')
    message_members, message_size = structure_layout(path, 'console_line_message')
    opcodes = {'remove': hid_opcode(ROOT / 'src/rcv_hid_cmd.h', 'RCV_HID_OP_REMOVE'),
               'clear': hid_opcode(ROOT / 'src/rcv_hid_cmd.h', 'RCV_HID_OP_CLEAR')}

    def symbol(name):
        matches = {value for key, value in symbols.items()
                   if key == name or key.startswith(name + '.')}
        if len(matches) != 1:
            raise ValueError(f'Ambiguous/missing ELF symbol {name}: {matches}')
        return matches.pop()

    # The linked ABI reports its own architecture-reserved stack bytes: a
    # K_THREAD_DEFINE stack object is the requested size plus that reservation.
    stats_stack = symbol('_k_thread_stack_esb_stats_thread_id')
    stats_thread = symbol('_k_thread_data_esb_stats_thread_id')[0]
    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for start in (0, 0x10000000, 0x20000000, 0x40000000, 0xE0000000, SCRATCH_BASE):
        machine.mem_map(start, 0x100000)
    for address, data in sections:
        machine.mem_write(address, data)
    # nRF52 FICR CODEPAGESIZE/CODESIZE for the real flash read driver.
    machine.mem_write(0x10000010, struct.pack('<II', PAGE_SIZE, 256))

    def word(address):
        return struct.unpack('<I', machine.mem_read(address, 4))[0]

    reserved = stats_stack[1] - word(stats_thread + offsets['init_stack_size'])
    if not 0 < reserved < stats_stack[1]:
        raise ValueError('Linked threads do not expose a usable stack reservation')
    entry_name, stack_name = (('handle_output_report', 'sys_work_q_stack')
                              if target == 'hid' else
                              ('console_thread', 'console_thread_stack'))
    entry = symbol(entry_name)[0]
    stack = symbol(stack_name)
    budget = stack[1] - reserved
    top = stack[0] + stack[1]

    machine.mem_write(REPORT, bytes([0xFE, 1, opcodes[command], 0]) + bytes(60))
    machine.mem_write(MESSAGE + message_members['epoch'], struct.pack('<I', 0))
    machine.mem_write(MESSAGE + message_members['line'], command.encode() + b'\0')

    flash_bytes = fixture([(ROSTER_ID, roster_record(len(SEEDED), SEEDED))]
                          + [(1000 + i, b'') for i in range(fillers)])
    flash_offset = None
    flash_writes = 0
    instructions = 0
    lowest = top
    deepest = []
    reached = Counter()
    modeled = Counter()
    recent = []
    phase = 'init'
    enqueue_seen = False

    def ret(value=0):
        machine.reg_write(UC_ARM_REG_R0, value & 0xffffffff)
        machine.reg_write(UC_ARM_REG_R1, 0)
        machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

    def stop():
        machine.reg_write(UC_ARM_REG_PC, STOP | 1)

    def hook(cpu, pc, size, unused):
        nonlocal lowest, deepest, instructions, flash_offset, flash_writes
        nonlocal phase, enqueue_seen
        instructions += 1
        if phase == 'command':
            sp = cpu.reg_read(UC_ARM_REG_SP)
            if sp < lowest:
                lowest, deepest = sp, recent[-16:]
        name = functions.get(pc)
        if not name:
            return
        base = name.split('.')[0]
        recent.append(name)
        del recent[:-24]
        if phase != 'init':
            reached[f'{phase}:{base}'] += 1
        if any(text in base for text in ('assert', 'fatal', 'fault', 'except')):
            raise RuntimeError(f'Unexpected firmware failure: {name}')
        if base == 'k_msgq_get':
            # Deliver one real console line, then stop once the command completed.
            if enqueue_seen:
                stop()
                return
            enqueue_seen = True
            cpu.mem_write(cpu.reg_read(UC_ARM_REG_R1), bytes(machine.mem_read(MESSAGE, message_size)))
            ret()
        elif base == 'flash_get_page_info':
            offset, info = cpu.reg_read(UC_ARM_REG_R1), cpu.reg_read(UC_ARM_REG_R2)
            if flash_offset is None:
                flash_offset = offset
                cpu.mem_write(offset, flash_bytes)
            if not flash_offset <= offset < flash_offset + len(flash_bytes):
                raise RuntimeError(f'Unexpected flash geometry offset {offset:#x}')
            for field, value in [('start_offset', offset // PAGE_SIZE * PAGE_SIZE),
                                 ('size', PAGE_SIZE), ('index', offset // PAGE_SIZE)]:
                machine.mem_write(info + geometry[field], struct.pack('<I', value))
            ret()
        elif base in ('flash_nrf_write', 'flash_nrf_erase'):
            offset = cpu.reg_read(UC_ARM_REG_R1)
            length = cpu.reg_read(UC_ARM_REG_R3 if base == 'flash_nrf_write' else UC_ARM_REG_R2)
            if flash_offset is None or not flash_offset <= offset <= offset + length <= flash_offset + len(flash_bytes):
                raise RuntimeError('Flash operation outside the NVS partition')
            if base == 'flash_nrf_write':
                data = bytes(cpu.mem_read(cpu.reg_read(UC_ARM_REG_R2), length))
                old = bytes(cpu.mem_read(offset, length))
                if any((a & b) != b for a, b in zip(old, data)):
                    raise RuntimeError('NVS attempted a zero-to-one flash write')
                cpu.mem_write(offset, data)
                if phase == 'command':
                    flash_writes += 1
            else:
                cpu.mem_write(offset, b'\xff' * length)
            ret()
        elif base in ('clocks_start', 'k_mutex_init', 'k_mutex_lock', 'k_mutex_unlock',
                      'z_impl_k_mutex_lock', 'z_impl_k_mutex_unlock',
                      'k_sched_lock', 'k_sched_unlock', 'k_msgq_purge', 'printk',
                      'device_is_ready', 'hid_write_packet_n', 'set_led'):
            ret()
        elif base.startswith('k_uptime_get'):
            ret(100)
        elif base.startswith('k_msleep'):
            ret()
        elif base == '__aeabi_read_tp':
            ret(SCRATCH_BASE + 0x200)
        elif base == 'z_log_msg_alloc':
            ret(SCRATCH_BASE + 0x800)
        elif base == 'z_log_msg_finalize':
            ret()
        else:
            return
        modeled[f'{phase}:{base}'] += 1

    machine.hook_add(UC_HOOK_CODE, hook)

    def run(function, args, sp, stop_at):
        machine.reg_write(UC_ARM_REG_SP, sp)
        machine.reg_write(UC_ARM_REG_LR, stop_at | 1)
        for reg, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            machine.reg_write(reg, value)
        machine.emu_start(function | 1, stop_at, count=INSTRUCTION_LIMIT)

    failures = []
    try:
        # Mount real NVS and publish the real roster from the fixture.
        run(symbol('sys_nvs_init')[0], (0, 0, 0, 0), INIT_TOP, STOP)
        if machine.reg_read(UC_ARM_REG_PC) != STOP or machine.reg_read(UC_ARM_REG_R0) != 0:
            raise RuntimeError('Real NVS mount did not return success')
        run(symbol('tracker_store_restore')[0], (0, 0, 0, 0), INIT_TOP, STOP)
        if (machine.reg_read(UC_ARM_REG_R0) != 0
                or machine.mem_read(symbol('stored_trackers')[0], 1)[0] != len(SEEDED)
                or list(struct.unpack('<16Q', machine.mem_read(symbol('stored_tracker_addr')[0], 128)))
                != SEEDED + [0] * (16 - len(SEEDED))):
            raise RuntimeError('Linked roster restore did not publish the fixture')

        phase = 'command'
        if target == 'hid':
            run(entry, (REPORT, 64, 1), top, STOP)
        else:
            run(entry, (0, 0, 0, 0), top, STOP)

        # The command must publish the new roster and commit it to flash.
        remaining = SEEDED[:-1] if command == 'remove' else []
        published = list(struct.unpack('<16Q', machine.mem_read(symbol('stored_tracker_addr')[0], 128)))
        if machine.mem_read(symbol('stored_trackers')[0], 1)[0] != len(remaining) or \
                published != remaining + [0] * (16 - len(remaining)):
            raise RuntimeError(f'Roster not published for {command}: {published}')
        if not flash_writes:
            raise RuntimeError('Roster transaction did not write flash')
        expected = roster_record(len(remaining), remaining)
        if not any(f'command:{name}' in reached for name in ('esb_pop_pair', 'esb_clear')):
            raise RuntimeError('Linked removal path was not executed')
        for required in ('tracker_roster_save', 'sys_write_checked', 'nvs_flash_data_al_wrt',
                         'nvs_flash_ate_wrt'):
            if not reached[f'command:{required}']:
                raise RuntimeError(f'Required real linked path not executed: {required}')
        collected = reached['command:nvs_gc'] and reached['command:nvs_flash_erase_sector']
        if fillers and not collected:
            raise RuntimeError('Near-full sector did not exercise NVS garbage collection')

        # Independent readback through linked NVS, outside the measured depth.
        phase = 'readback'
        run(symbol('sys_read_checked')[0], (ROSTER_ID, READBACK, len(expected), 0), INIT_TOP, STOP)
        if (machine.reg_read(UC_ARM_REG_PC) != STOP
                or machine.reg_read(UC_ARM_REG_R0) != len(expected)
                or bytes(machine.mem_read(READBACK, len(expected))) != expected):
            raise RuntimeError('NVS readback differs from the expected committed roster')
    except (UcError, RuntimeError, ValueError) as exc:
        failures.append(str(exc))
    used = top - lowest
    if used > budget:
        failures.append(f'Measured stack {used} exceeds linked budget {budget}')
    if used + EXCEPTION_RESERVE > budget:
        failures.append(f'Measured stack {used} plus basic exception frame {EXCEPTION_RESERVE} '
                        f'exceeds linked budget {budget}')
    return {
        'target': target, 'command': command, 'deleted_filler_ates': fillers,
        'linked_thread': stack_name,
        'nvs_garbage_collection_exercised': bool(reached['command:nvs_gc']),
        'linked_stack_object_bytes': stack[1], 'architecture_reserved_bytes': reserved,
        'linked_stack_budget': budget, 'measured_command_stack_lower_bound': used,
        'basic_exception_frame_reserve': EXCEPTION_RESERVE,
        'headroom_after_basic_exception_frame': budget - used - EXCEPTION_RESERVE,
        'command_flash_writes': flash_writes,
        'deepest_recent_functions': deepest, 'executed_functions': dict(reached),
        'modeled_leaves': dict(modeled), 'instructions': instructions,
        'failures': failures,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, help='Unstripped nRF52 receiver ELF with DWARF')
    args = parser.parse_args()
    image = load_image(args.elf)
    results = [replay(args.elf, image, target, command, fillers)
               for target, command, fillers in CASES]
    print(json.dumps({'elf': str(args.elf), 'cases': results}, indent=2))
    return int(any(result['failures'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
