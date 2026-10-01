#!/usr/bin/env python3
"""Check real ARM ELF/map output, including the safety-critical stack reservation.

Without arguments, run red-capable verifier regressions. With a board and build
path, inspect that actual firmware (no mock SDK or nominal CMake-only check).
"""

import copy
from pathlib import Path
import re
import subprocess
import struct
import sys
import tempfile
import unittest


RAM_START = 0x20000000
RAM_LENGTH = {"pico": 0x40000, "pico2": 0x80000}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def verify(board, symbols, sections, memory):
    end = RAM_START + RAM_LENGTH[board]
    bottom = end - 0x4000
    value = lambda name: symbols[name][0]
    require(memory == (RAM_START, RAM_LENGTH[board]), "wrong platform RAM")
    require(value("__StackTop") == end, "core0 stack top")
    require(value("__StackBottom") == bottom, "core0 stack must be 16 KiB")
    require(value("__StackLimit") == bottom, "allocator stack limit")
    require(value("_initial_vector_sp") == end, "startup vector stack pointer")
    require(sections[".stack_dummy"] == (bottom, 0x4000, "NOBITS"),
            "core0 stack must be a real 16 KiB RAM reservation")
    require(RAM_START <= value("__end__") <= value("__HeapLimit") <= bottom,
            "static data/heap/stack overlap")
    core1, size, kind = symbols["core1_stack"]
    bss, bss_size, bss_kind = sections[".bss"]
    require(size == 0x2000 and kind.lower() == "b" and bss_kind == "NOBITS",
            "core1 must have an 8 KiB BSS stack")
    require(bss <= core1 and core1 + size <= bss + bss_size <= value("__end__"),
            "core1 stack outside BSS/static data")
    require(core1 % 8 == 0, "core1 stack alignment")
    for name, (address, size, _) in sections.items():
        if name != ".stack_dummy" and RAM_START <= address < end and size:
            require(address + size <= bottom, f"{name} overlaps core0 stack")
    # SDK 2.3's TLS metadata and per-core storage must survive the override.
    tls_size = value("__tls_size")
    require(value("__tbss_offset") >= value("__tdata_size"), "TLS data overlap")
    require(tls_size == value("__tbss_offset") + value("__tbss_size"), "TLS size/layout")
    require(value("__tls_align") > 0, "TLS alignment missing")
    for name in (".tls0", ".tls1"):
        address, size, kind = sections[name]
        require((size == 0 or kind == "NOBITS") and size >= value("__tlsX_size_align"),
                f"{name} storage missing")
        require(RAM_START <= address and address + size <= value("__end__"),
                f"{name} outside static RAM")


def inspect(board, build):
    elf = build / "firmware.elf"
    nm = subprocess.check_output(
        ["arm-none-eabi-nm", "-S", "--defined-only", str(elf)], text=True
    )
    symbols = {}
    core1_candidates = []
    for line in nm.splitlines():
        parts = line.split()
        if len(parts) == 4:
            address, size, kind, name = parts
            symbols[name] = (int(address, 16), int(size, 16), kind)
            if name == "core1_stack":
                core1_candidates.append(symbols[name])
        elif len(parts) == 3:
            address, kind, name = parts
            symbols[name] = (int(address, 16), 0, kind)
    headers = subprocess.check_output(
        ["arm-none-eabi-readelf", "-SW", str(elf)], text=True
    )
    sections = {}
    elf_bytes = elf.read_bytes()
    for match in re.finditer(
        r"\[\s*\d+\]\s+(\S+)\s+(\S+)\s+([0-9a-fA-F]+)\s+"
        r"([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\S+\s+([A-Z]*)", headers
    ):
        name, kind, address, offset, size, flags = match.groups()
        address, offset, size = (int(x, 16) for x in (address, offset, size))
        if "A" in flags or name in (".tls0", ".tls1"):
            sections[name] = (address, size, kind)
        vectors = symbols["__vectors"][0]
        if "A" in flags and kind == "PROGBITS" and address <= vectors < address + size:
            sp = struct.unpack_from("<I", elf_bytes, offset + vectors - address)[0]
            symbols["_initial_vector_sp"] = (sp, 0, "A")
    # The SDK has a separate default scratch-bank stack with the same local
    # symbol name. Check the application's explicit BSS stack, not that fallback.
    bss, bss_size, _ = sections[".bss"]
    explicit = [entry for entry in core1_candidates if bss <= entry[0] < bss + bss_size]
    require(len(explicit) == 1, "expected one explicit core1 BSS stack")
    symbols["core1_stack"] = explicit[0]
    map_text = (build / "firmware.elf.map").read_text()
    region = re.search(r"^RAM\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)", map_text, re.M)
    require(region is not None, "map has no RAM region")
    memory = tuple(int(x, 16) for x in region.groups())
    require("core0 stack must reserve 16 KiB" in map_text, "map lacks stack assertion")
    verify(board, symbols, sections, memory)
    print(f"{board}: ELF/map verified; core0=16384, core1 BSS=8192, "
          f"heap headroom={symbols['__HeapLimit'][0] - symbols['__end__'][0]}, "
          f"TLS={symbols['__tls_size'][0]}")


class MemoryVerifierTests(unittest.TestCase):
    def fixture(self, board):
        end = RAM_START + RAM_LENGTH[board]
        symbols = {name: (value, 0, "A") for name, value in {
            "__StackTop": end, "__StackBottom": end - 0x4000,
            "__StackLimit": end - 0x4000, "_initial_vector_sp": end,
            "__end__": RAM_START + 0x3000, "__HeapLimit": end - 0x4000,
            "__tls_size": 8, "__tdata_size": 4, "__tbss_size": 4,
            "__tbss_offset": 4, "__tls_align": 4, "__tlsX_size_align": 8,
        }.items()}
        symbols["core1_stack"] = (RAM_START, 0x2000, "b")
        sections = {".stack_dummy": (end - 0x4000, 0x4000, "NOBITS"),
                    ".bss": (RAM_START, 0x2000, "NOBITS"),
                    ".tls0": (RAM_START + 0x2000, 8, "NOBITS"),
                    ".tls1": (RAM_START + 0x2008, 8, "NOBITS")}
        return symbols, sections, (RAM_START, RAM_LENGTH[board])

    def test_linker_edits_fail_closed(self):
        module = Path(__file__).resolve().parents[1] / "cmake/control_memory.cmake"
        for text, expected_success in (("one token end", True), ("no match", False),
                                       ("token token", False)):
            with tempfile.TemporaryDirectory() as temp:
                script = Path(temp) / "probe.cmake"
                script.write_text(f'include("{module}")\nset(fragment "{text}")\n'
                                  'control_replace_once(fragment "token" "replacement")\n')
                result = subprocess.run(["cmake", "-P", str(script)], capture_output=True)
                self.assertEqual(result.returncode == 0, expected_success, result.stderr)

    def test_both_platforms(self):
        for board in RAM_LENGTH:
            verify(board, *self.fixture(board))

    def test_emulated_tls_needs_no_native_storage(self):
        symbols, sections, memory = self.fixture("pico")
        for name in ("__tls_size", "__tdata_size", "__tbss_size", "__tbss_offset",
                     "__tlsX_size_align"):
            symbols[name] = (0, 0, "A")
        for name in (".tls0", ".tls1"):
            sections[name] = (RAM_START + 0x2000, 0, "PROGBITS")
        verify("pico", symbols, sections, memory)

    def test_rejects_symbol_only_stack_and_memory_regressions(self):
        for board in RAM_LENGTH:
            for name, value in (("__StackBottom", RAM_START),
                                ("__HeapLimit", RAM_START + RAM_LENGTH[board]),
                                ("__StackLimit", RAM_START), ("_initial_vector_sp", RAM_START),
                                ("__tls_size", 0)):
                symbols, sections, memory = self.fixture(board)
                symbols[name] = (value, 0, "A")
                with self.subTest(board=board, symbol=name), self.assertRaises(ValueError):
                    verify(board, symbols, sections, memory)
            symbols, sections, memory = self.fixture(board)
            for name, replacement in ((".stack_dummy", (RAM_START, 0x4000, "NOBITS")),
                                      (".tls1", (RAM_START, 0, "NOBITS"))):
                changed = copy.deepcopy(sections)
                changed[name] = replacement
                with self.subTest(board=board, section=name), self.assertRaises(ValueError):
                    verify(board, symbols, changed, memory)
            symbols["core1_stack"] = (RAM_START, 0x800, "b")
            with self.assertRaises(ValueError):
                verify(board, symbols, sections, memory)


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] in RAM_LENGTH:
        inspect(sys.argv[1], Path(sys.argv[2]))
    else:
        unittest.main(verbosity=2)
