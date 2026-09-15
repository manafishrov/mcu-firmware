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
printf '\nEmbedded development identity:\n'
arm-none-eabi-strings build/pico/firmware.elf | grep '^pico-control-dev:'
printf '\nELF section totals (bytes):\n'
arm-none-eabi-size build/pico/firmware.elf
printf '\nLinker memory boundaries:\n'
grep -E '^\.bss |^\.data |^\.heap |^\.stack_dummy|__HeapLimit =|__StackBottom =' build/pico/firmware.elf.map
printf '\nCompiler/tool versions:\n'
arm-none-eabi-gcc --version | head -n 1
cmake --version | head -n 1
