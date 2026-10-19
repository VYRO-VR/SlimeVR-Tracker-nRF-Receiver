#!/usr/bin/env python3
"""Replay the linked ARM stats thread, including LTO TDMA and cbprintf code.

Run with an unstripped receiver zephyr.elf containing DWARF:
    uv run --with unicorn --with pyelftools run_stack.py PATH/zephyr.elf

Only uptime, sleep, scheduler lock/unlock, and log allocation/finalization are
replaced. Sleep supplies online-tracker evidence, not simulated RF packets.
The measured maximum is a LOWER BOUND: thread entry, exception/context frames,
interrupts, and the replaced leaves are excluded. Passing is not hardware stack
safety proof; exceeding the linked usable stack budget is a definite failure.
No source matching, guessed guard size, or added safety margin is used.
"""

import argparse
import json
from pathlib import Path
import struct

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_SP,
)


DURATION_MS = 20_000
INSTRUCTION_LIMIT = 1_000_000
SCRATCH_BASE = 0x21000000
SCRATCH_SIZE = 0x100000


def static_thread_offsets(elf):
    """Use this ELF's ABI, not the host ABI or a guessed Zephyr struct layout."""
    required = {'init_stack', 'init_stack_size', 'init_entry'}
    for unit in elf.get_dwarf_info().iter_CUs():
        for die in unit.iter_DIEs():
            name = die.attributes.get('DW_AT_name')
            if (die.tag != 'DW_TAG_structure_type' or name is None
                    or name.value != b'_static_thread_data'):
                continue
            offsets = {}
            for member in die.iter_children():
                name = member.attributes.get('DW_AT_name')
                location = member.attributes.get('DW_AT_data_member_location')
                if name and location and isinstance(location.value, int):
                    offsets[name.value.decode()] = location.value
            if required <= offsets.keys():
                return offsets
    raise ValueError('ELF needs DWARF for _static_thread_data member offsets')


def load_image(path):
    with path.open('rb') as stream:
        elf = ELFFile(stream)
        if elf.elfclass != 32 or not elf.little_endian or elf['e_machine'] != 'EM_ARM':
            raise ValueError('Expected a little-endian 32-bit ARM receiver ELF')
        symtab = elf.get_section_by_name('.symtab')
        if symtab is None:
            raise ValueError('ELF must retain its symbol table')
        symbols = {symbol.name: (int(symbol['st_value']), int(symbol['st_size']))
                   for symbol in symtab.iter_symbols()}
        functions = {int(symbol['st_value']) & ~1: symbol.name
                     for symbol in symtab.iter_symbols()
                     if symbol['st_info']['type'] == 'STT_FUNC'
                     and symbol['st_shndx'] != 'SHN_UNDEF'}
        sections = [(int(section['sh_addr']), section.data())
                    for section in elf.iter_sections()
                    if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS'
                    and section['sh_size']]
        return symbols, functions, sections, static_thread_offsets(elf)


def replay(image, trackers):
    symbols, functions, sections, offsets = image
    machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    # The supported nRF52 receiver image address spaces. Peripheral memory is
    # inert: no RF/USB device emulation or timing claims are made by this test.
    for start in (0, 0x20000000, 0x40000000, 0xE0000000, SCRATCH_BASE):
        machine.mem_map(start, 0x100000)
    for address, data in sections:
        machine.mem_write(address, data)

    def word(address):
        return struct.unpack('<I', machine.mem_read(address, 4))[0]

    def put(name, value, size=4):
        machine.mem_write(symbols[name][0], value.to_bytes(size, 'little'))

    stack, storage = symbols['_k_thread_stack_esb_stats_thread_id']
    thread_data = symbols['_k_thread_data_esb_stats_thread_id'][0]
    budget = word(thread_data + offsets['init_stack_size'])
    entry = word(thread_data + offsets['init_entry'])
    if word(thread_data + offsets['init_stack']) != stack:
        raise ValueError('Static thread initializer does not reference its stack')
    if entry != symbols['esb_stats_thread'][0] or not 0 < budget <= storage:
        raise ValueError('Inconsistent linked stats thread initializer')
    # Zephyr places the usable downward-growing stack at the top of its storage;
    # init_stack_size already excludes the architecture-reserved guard bytes.
    top = stack + storage
    machine.reg_write(UC_ARM_REG_SP, top)
    machine.reg_write(UC_ARM_REG_LR, 0x80001)
    put('stored_trackers', trackers, 1)
    now = 0
    lowest = top
    instructions = 0
    packages = 0
    tdma_packages = 0
    finalized = 0
    allocated = SCRATCH_BASE
    trace = []
    deepest = []
    completed = False

    def ret(value=0):
        machine.reg_write(UC_ARM_REG_R0, value)
        machine.reg_write(UC_ARM_REG_R1, 0)
        machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

    def hook(cpu, pc, size, user_data):
        nonlocal now, lowest, instructions, packages, tdma_packages, finalized
        nonlocal allocated, deepest, completed
        instructions += 1
        sp = cpu.reg_read(UC_ARM_REG_SP)
        if sp < lowest:
            lowest = sp
            deepest = trace[-12:]
        name = functions.get(pc)
        if name is None:
            return
        trace.append(name)
        if len(trace) > 24:
            del trace[:-12]
        if 'assert' in name or 'fatal' in name:
            raise RuntimeError(f'Unexpected firmware failure: {name}')
        if name.startswith('k_uptime_get'):
            ret(now)
        elif name.startswith('k_msleep'):
            delay = cpu.reg_read(UC_ARM_REG_R0)
            if not 0 < delay <= DURATION_MS:
                raise RuntimeError(f'Unexpected stats sleep: {delay}')
            # Run every wake through t=20s, then stop at the next sleep without
            # pretending that its future wake has already executed.
            if now + delay > DURATION_MS:
                completed = now == DURATION_MS
                cpu.emu_stop()
                return
            now += delay
            if trackers:
                put('tdma_shadow_live_evidence_mask', (1 << trackers) - 1)
                cpu.mem_write(symbols['tdma_shadow_last_seen_ms'][0], struct.pack(
                    '<16I', *([now + 1] * trackers + [0] * (16 - trackers))))
            ret()
        elif name in ('k_sched_lock', 'k_sched_unlock'):
            ret()
        elif name.startswith('z_log_msg_finalize'):
            finalized += 1
            ret()
        elif name == 'z_log_msg_alloc':
            # Zephyr allocator's argument is a count of 32-bit words. Keep each
            # allocation separate while executing the real package copy code.
            length = cpu.reg_read(UC_ARM_REG_R0) * 4
            if not length or allocated + length > SCRATCH_BASE + SCRATCH_SIZE:
                raise RuntimeError('Unexpected/excessive log allocation')
            result = allocated
            allocated += (length + 7) & ~7
            ret(result)
        elif name.startswith('cbprintf_package_convert'):
            # ARM cbprintf package header occupies one word, followed by the
            # format pointer. Inspect executed packages, never source strings.
            packages += 1
            package = cpu.reg_read(UC_ARM_REG_R0)
            format_address = word(package + 4)
            prefix = bytes(cpu.mem_read(format_address, len(b'TDMA reconfig:')))
            if prefix == b'TDMA reconfig:':
                tdma_packages += 1

    machine.hook_add(UC_HOOK_CODE, hook)
    error = None
    try:
        machine.emu_start(entry, 0x80000, count=INSTRUCTION_LIMIT)
    except (UcError, RuntimeError) as exc:
        error = str(exc)
    used = top - lowest
    failures = []
    if error:
        failures.append(error)
    if not completed:
        failures.append('Did not execute the full 20-second startup scenario')
    if trackers and (not tdma_packages or not finalized):
        failures.append('Tracker scenario did not package/finalize a TDMA reconfiguration log')
    if used > budget:
        failures.append(f'Measured stack {used} exceeds linked budget {budget}')
    active_mask = word(symbols['tdma_active_mask'][0])
    active_count = machine.mem_read(symbols['tdma_dynamic_active_count'][0], 1)[0]
    if active_mask != (1 << trackers) - 1 or active_count != trackers:
        failures.append(f'Wrong final TDMA roster: mask={active_mask:#x}, count={active_count}')
    return {
        'trackers': trackers,
        'elapsed_ms': now,
        'tdma_active_mask': active_mask,
        'tdma_dynamic_active_count': active_count,
        'linked_stack_budget': budget,
        'stack_storage_bytes': storage,
        'measured_thread_stack': used,
        'headroom_excluding_entry_interrupts_and_stubbed_leaves': budget - used,
        'cbprintf_conversion_calls': packages,
        'tdma_reconfig_conversion_calls': tdma_packages,
        'finalized_logs': finalized,
        'instructions': instructions,
        'deepest_recent_functions': deepest,
        'failures': failures,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, help='Unstripped linked receiver ELF with DWARF')
    args = parser.parse_args()
    image = load_image(args.elf)
    # Independent machines preserve linked startup state for every roster size.
    results = [replay(image, trackers) for trackers in (0, 1, 16)]
    print(json.dumps({'elf': str(args.elf), 'cases': results}, indent=2))
    return int(any(result['failures'] for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
