# Read-only BMI270 SPI comparison probe

This is a separate **diagnostic image for the original RP2040 Pico**, not the
ROV controller. Wiring is confirmed. The experiment compares transfer methods
on those same pins; it does not assume a wiring fault.

It does not initialize the BMI270, upload its configuration image, write any
IMU register, run AHRS/PID, or exercise motor protocols. **It is not evidence of
500 Hz control or IMU sampling performance.** No production source, root build
configuration, release version, or normal controller UF2 is changed by this
project. Only the coordinator may flash the probe and restore the saved image.

## Pins and safety

| GPIO | Use |
| --- | --- |
| 6, 7, 8, 9, 18, 19, 20, 21 | SIO outputs, latch low and output override forced low at the start of `main`; never changed afterward |
| 10 | SPI1 SCK, then SIO SCK for the bitbang comparison |
| 11 | SPI1 MOSI, then SIO MOSI |
| 12 | SPI1 MISO / SIO input only; never an output |
| 13 | SIO chip select, idle high |

No PWM, PIO, motor protocol, controller, Bosch driver, UART stdio, or LED code
is linked into the diagnostic application. USB D+/D- and flash use their normal
dedicated pins. USB reset activity LEDs are disabled and cannot be selected by
a host reset request. This safety description applies while the probe runs,
not during flashing, BOOTSEL, or execution of the restored controller.

Both hardware-SPI transfer methods execute on core 1. Core 0 owns chip select,
configuration and the USB report. A 2 ms core-0 request deadline contains even
a stalled SDK blocking call: stop issuing requests, disable SPI, make its pins
SIO idle, and retain USB service. The worker never writes GPIO or chip select,
so a late worker completion cannot reassert CS. No MCU or core reset is used
on timeout. The inherited watchdog is disabled; the probe never enables it.
The current bytewise transfer also retains its own 500 us deadline.

After USB CDC DTR is asserted, exactly one run occurs. There is a 1 s run budget,
fixed transaction counts, and a separate 5 s USB-report budget. Timing bounds
exclude interrupt preemption and local SDK peripheral/boot initialization.
No output waits for a sensor response or for data-ready. There is no rerun on
CDC reconnect. On completion or transfer timeout, CS is high, SCK/MOSI low,
MISO is an input with pulls disabled, and motors remain forced low indefinitely.

Core 0 reserves the full 4 KiB SCRATCH_Y bank (`0x20041000..0x20042000`);
core 1 reserves 2 KiB (`0x20040800..0x20041000`). The linker map verifies these
ranges; this is not just an unreserved stack-size constant. Release compiler
`.su` evidence gives `main=136`, `report_probe=680` (including the 512-byte line),
`snprintf=32`, `_vsnprintf=184`, `_ntoa_format=56`, `_out_rev=40` bytes. That
identified integer-formatting chain totals 1128 bytes before small leaf and
exception overhead, leaving 2968 bytes within the core-0 reservation. Relevant
USB frames include worker IRQ 32, `tud_task_ext` 104, USB IRQ handler 8 and buffer
handler 56 bytes, plus hardware exception frames. Core 1's worker frame is 64
bytes and SDK SPI transfer frame 28 bytes. There is no recursive application
code or variable-length stack allocation. These are static budget checks,
not measured hardware stack high-water marks; `.su` files and the linker map
remain in `build/spi_probe` for review.

The Pico SDK USB vendor reset interface remains available for the coordinator's
restore operation. Baud-rate-triggered reset is disabled. The probe never
resets itself, flashes anything, or accesses a remote machine.

## Experiment

Each variant performs three pairs of `80 00 00` read transactions, keeping CS
low across all three bytes. Each pair's first `select` read is discarded for
identification; the subsequent `id` read's **third RX byte** is CHIP_ID (expected
`24`). At least 500 us of CS-high time separates every transaction, exceeding
the required 450 us between selection and identification.

1. Current bytewise FIFO register algorithm at requested 4 MHz, mode 0.
2. Unmodified SDK `spi_write_read_blocking` at the same speed and mode.
3. Matched bytewise/SDK pairs at 100 kHz and 1 MHz, still mode 0.
4. Matched bytewise/SDK pairs at 4 MHz, mode 3.
5. Fixed-loop SIO bitbang, mode 0, nominal 50 kHz, on the same four pins.
6. Optional, separately labelled SDK 4 MHz/mode-0 weak-MISO-pull-up and
   weak-pull-down-repeat variants (`SPI_PROBE_COMPARE_PULLS=ON`).

The first nine variants retain the production weak MISO pulldown. The optional
last two are bias experiments, not a wiring diagnosis. **RX following the pull
is evidence of an undriven/weakly driven input. Unchanged `00` does not prove
active drive:** external bias, a short, or input thresholds can also hold it low.
The probe deliberately reports observations without classifying their cause.

The bytewise algorithm is copied from production commit
`b817d1d3da143bbb1d56ce4a86ff0a15e5ae1f3c`, whose `src/imu/bmi270_sensor.c` SHA-256
is `dd743f611485d1fb7b3a47f25555be453e4fd0e3b3b305265a79d4e7deebd306`.
Unlike production, the probe retains command-phase RX instead of discarding it.
No production file is included in or modified by the diagnostic build.

## Reading the report

The USB product is `Manafish BMI270 read-only SPI probe`; binary program name is
`manafish-bmi270-read-only-spi-probe`. A `SPI_PROBE` header includes the exact
`probe.c` source hash and record count. With pulls enabled, a full run has
66 records (54 without). `END_SPI_PROBE` marks completion of the report.
Missing that marker means an incomplete capture, even if the pins were idled.

Each record includes all three RX bytes, transfer method, repetition and phase,
requested rate, SDK divider readback (`actual_baud`), mode, pull, duration,
CS-high gap, pinmux for GPIO10..13, raw pad-level bitmaps, the complete GPIO12
IO_BANK0 STATUS register before/after, and masked motor input levels.
`complete=1` means the transfer finished, **not** that the sensor is healthy or
that CHIP_ID matched. On an incomplete transfer, `ee` may mark an unread byte;
no RX byte in an incomplete record should be treated as valid identification.

Pad bitmaps use bit0=GPIO10, bit1=GPIO11, bit2=GPIO12, bit3=GPIO13. GPIO12 raw
STATUS includes INFROMPAD at bit17. These are idle snapshots around the transfer,
**not physical waveform proof**. Hardware `actual_baud` comes from the configured
clock divider, not a scope measurement. SIO reports `actual_baud=0` because its
50 kHz is nominal; call overhead and interrupts affect the actual waveform.
Transaction durations include the worker dispatch/wait overhead for both
hardware methods. They are not controller execution-rate measurements.

## Build and host-only gates

Use this repository's locked Nix shell (Pico SDK 2.1.1, ARM GCC 14.2.rel1).
From a shell with `repo` set to the absolute MCU worktree path:

```sh
nix develop "$repo" --command bash "$repo/scripts/spi_probe/check.sh"
```

The script builds **only** `build/spi_probe/spi_probe.uf2`, performs scoped
format/tidy checks and strict offline tests (pull comparisons both disabled and
enabled, plus ASan/UBSan), then runs repository format/test/lint gates. It uses
`make -o build-pico lint-check` against the existing production compile database
so the lint prerequisite cannot regenerate the normal controller UF2. It
requires that existing database and normal UF2, and verifies the normal UF2
hash is unchanged. It never flashes, reboots, or contacts hardware.

Equivalent dedicated build commands inside that Nix shell:

```sh
cmake -S "$repo/scripts/spi_probe" -B "$repo/build/spi_probe" \
  -DPICO_BOARD=pico -DCMAKE_BUILD_TYPE=Release -DSPI_PROBE_COMPARE_PULLS=ON
cmake --build "$repo/build/spi_probe" -j4
sha256sum "$repo/scripts/spi_probe/probe.c" "$repo/build/spi_probe/spi_probe.uf2"
```

`tests/test_probe.c` includes the actual diagnostic source with a recording fake
SDK and cooperative worker scheduler. It checks every output against the GPIO
allowlist, motor-low invariants, the read-only byte/bit trace, all variants,
three selection/ID pairs, CS gaps, all three received bytes, DTR gating,
one-shot behavior, timeout cleanup, USB backpressure, and raw pull reporting.
It does not replace physical bench measurements or verify silicon clock edges.

## Coordinator-owned install and restore

Do not flash until the scoped and existing repository gates pass and the
coordinator confirms the diagnostic ELF/UF2 hashes. Keep ESCs disconnected.
The coordinator already owns the immutable full-flash backup and verified
`b817d1d` restore UF2. The latter's SHA-256 is:

`21f22a2eeb34bba47699468aadd3d0d50d284711db1f7438f36ef2749f683339`

Preserve those backups; do not regenerate a normal controller UF2 as a restore
substitute. The coordinator alone installs this explicitly named probe,
asserts DTR to collect one report, then uses the SDK USB reset interface to
restore the verified image. No install/restore automation is supplied here.
