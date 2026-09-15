#!/usr/bin/env bash
# Host-only build/check entry point. No hardware, flash, reboot, or remote commands.
set -euo pipefail
probe=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd -- "$probe/../.." && pwd)
build="$repo/build/spi_probe"
: "${PICO_SDK_PATH:?Run through nix develop from the MCU repository}"
normal="$repo/build/pico/firmware.uf2"
[[ -f "$normal" && -f "$repo/build/pico/compile_commands.json" ]] || {
    echo 'Existing production UF2 and compile database required; this check never rebuilds them.' >&2
    exit 1
}
normal_before=$(sha256sum "$normal" | cut -d ' ' -f1)
verify_normal_unchanged() {
    local status=$?
    local normal_after
    normal_after=$(sha256sum "$normal" | cut -d ' ' -f1)
    if [[ "$normal_before" != "$normal_after" ]]; then
        echo 'ERROR: production UF2 changed during diagnostic checks' >&2
        exit 1
    fi
    printf 'Production UF2 unchanged: %s\n' "$normal_after"
    exit "$status"
}
trap verify_normal_unchanged EXIT

clang-format --dry-run --Werror "$probe/probe.c" "$probe/tests/test_probe.c" "$probe/tests/mock_sdk.h"
cmake -S "$probe" -B "$build" -DPICO_BOARD=pico -DCMAKE_BUILD_TYPE=Release \
    -DSPI_PROBE_COMPARE_PULLS=ON
cmake --build "$build" -j4
clang-tidy "$probe/probe.c" --warnings-as-errors='*' -p "$build" \
    --extra-arg="-idirafter$(arm-none-eabi-gcc -print-file-name=include)" \
    --extra-arg="-I$(arm-none-eabi-gcc -print-sysroot)/include"
for pulls in 0 1; do
    cc -std=c11 -Wall -Wextra -Werror -O2 -DSPI_PROBE_COMPARE_PULLS="$pulls" \
        "$probe/tests/test_probe.c" -o "$build/test_probe_$pulls"
    "$build/test_probe_$pulls"
done
cc -std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined \
    "$probe/tests/test_probe.c" -o "$build/test_probe_sanitized"
"$build/test_probe_sanitized"

# Existing repository gates, but do NOT regenerate the normal controller UF2.
make -C "$repo" format-check
make -C "$repo" -o build-pico lint-check
make -C "$repo" test

git -C "$repo" diff --check
printf '\nDiagnostic source / ELF / UF2 hashes:\n'
sha256sum "$probe/probe.c" "$build/spi_probe.elf" "$build/spi_probe.uf2"
