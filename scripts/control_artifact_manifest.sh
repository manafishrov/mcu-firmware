#!/usr/bin/env bash
# Run from the MCU development shell after the final build; never touches hardware.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
printf 'task=pico-spi-attitude-500hz\nboard=pico\n'
printf 'source_commit=%s\n' "$(git rev-parse HEAD)"
printf 'source_branch=%s\n' "$(git branch --show-current)"
if [[ -n $(git status --porcelain) ]]; then
  printf 'source_dirty=true\n'
else
  printf 'source_dirty=false\n'
fi
printf '\nArtifact SHA-256:\n'
sha256sum build/pico/firmware.uf2 build/pico/firmware.elf
printf '\nEmbedded control identity:\n'
# Read the actual CAPS constant, not a possibly different legacy version string.
# A stripped/unknown ELF must still produce the remaining size/map evidence.
python3 - build/pico/firmware.elf <<'PY'
import re
import subprocess
import sys

elf = sys.argv[1]
symbols = subprocess.check_output(["arm-none-eabi-nm", "-S", elf], text=True)
matches = [fields for line in symbols.splitlines() if len(fields := line.split()) == 4
           and fields[2].lower() == "r" and re.fullmatch(r"identity\.\d+", fields[3])]
if len(matches) != 1:
    print("unknown (CAPS identity symbol missing or ambiguous)")
    sys.exit(0)
address, size = (int(value, 16) for value in matches[0][:2])
if not 1 < size <= 768:
    print("unknown (unsupported CAPS identity size)")
    sys.exit(0)
dump = subprocess.check_output([
    "arm-none-eabi-objdump", "-s", f"--start-address={address}",
    f"--stop-address={address + size}", elf,
], text=True)
data = bytearray()
for line in dump.splitlines():
    fields = line.split()
    if not fields or not re.fullmatch(r"[0-9a-fA-F]+", fields[0]):
        continue
    if int(fields[0], 16) != address + len(data):
        continue
    for word in fields[1:5]:
        if not re.fullmatch(r"(?:[0-9a-fA-F]{2}){1,4}", word):
            break
        data.extend(bytes.fromhex(word)[:size - len(data)])
        if len(data) == size:
            break
if len(data) == size and data[-1:] == b"\0" and all(32 <= byte < 127 for byte in data[:-1]):
    print(data[:-1].decode("ascii"))
else:
    print("unknown (unreadable CAPS identity constant)")
PY
printf '\nELF section totals (bytes):\n'
arm-none-eabi-size build/pico/firmware.elf
printf '\nLinker memory boundaries:\n'
grep -E '^\.bss |^\.data |^\.heap |^\.stack_dummy|__HeapLimit =|__StackBottom =' build/pico/firmware.elf.map
printf '\nCompiler/tool versions:\n'
arm-none-eabi-gcc --version | head -n 1
cmake --version | head -n 1
