# Pico SPI control handoff

Task: `pico-spi-attitude-500hz`. Baseline MCU `47c503e`; branch
`feat/pico-spi-attitude-500hz`. This is a development build, not a release.
No version bump, push, PR, tag, release or hardware operation was performed by
the MCU implementation agents.

**Do not flash the uncorrected `7f152f8` commit or an artifact identified only
by that commit.** The safety corrections verified below supersede
`7f152f8f0473ea8011ad7799bb4b0f86a6354a25`. The recorded dirty-build hashes are
pre-commit evidence, not the selected hardware artifacts. Rebuild the committed
tree and generate its exact artifact manifest before hardware use. The
coordinator records the selected commit and image hash in the paired firmware
repository's validation ledger; this handoff does not authorize flashing.

## Source and compatibility

- `PICO_CONTROL_PROTOCOL.md`: wire layouts, settings atomicity, maintenance and
  exact cadence/frame semantics. Settings are 628 bytes, ATTITUDE 84, IMU 28.
- `src/control/controller.*`: SDK-independent mathematical port. AHRS/attitude
  PID and allocation run at 500Hz; desired targets/depth PID advance once per
  accepted 60Hz command. Nullspace solves at 500Hz but decays only on command ticks.
- `src/control/protocol.*`: bounded explicit-endian codec, CRC32C and settings
  decode/validation. No native-struct serialization.
- `src/control/runtime.*`: core ownership, bounded queues, transactions,
  leases, telemetry and hardware-timer/watchdog safety.
- `src/usb_rx.*`: USB worker IRQ timestamping, preserving arrival time through
  main-loop stalls. `src/usb_tx.*`: bounded whole-packet priority queues;
  blocked host reads cannot block the controller or corrupt a partly sent frame.
- `src/imu/bmi270_sensor.*`: raw Bosch SPI wrapper. Official driver/config image
  in `third_party/bmi270`, pinned to commit
  `41129fcfe39c583ee5462d79195741945d51c1fe`; preserve its BSD-3-Clause notices
  in source and binary distributions.

Python must capability-probe through the old fixed C5 envelope before sending
extended packets. New code accepts old tools in legacy mode, but negotiated
control rejects legacy nonneutral/config/ESC packets. The CRC-protected 0x25
gate enters sticky neutral maintenance for unchanged ESC upload protocols.
See the protocol document before attempting ESC maintenance.

Build identity is `pico-control-dev:<git-sha>[-dirty]` in capability telemetry;
the historical release reply is not fabricated. The development Pi service
must set `MANAFISH_PICO_CONTROL_DEVELOPMENT=1` to opt out of bundled-UF2
reconciliation, and the host must also verify this marker and API capability.
Remove that environment override when restoring the original deployment.

## Local verification

From this MCU worktree, using its Nix dev shell:

```sh
make format-check
make lint-check
make test
make build-pico
make build-pico2
CONTROL_RUNTIME_SANITIZERS=address,undefined python3 tests/test_control_runtime.py
arm-none-eabi-size build/pico/firmware.elf
sha256sum build/pico/firmware.uf2
```

`make test` retains the original startup/current-reporting regressions and adds
pure controller/codec Unity tests and the real-Bosch SPI emulator. Cross-language
oracles live in the paired Python repository at
`tests/test_pico_differential.py`, with frozen original Python source snapshots.
Set `PICO_CONTROLLER_SOURCE` to this worktree's `src/control/controller.c` when
running those tests on another machine.

Core0 has a real 16KiB stack reservation at the top of main RAM, with the heap
limit below it. Core1 has an explicit 8KiB BSS stack. The generated linker map
is `build/pico/firmware.elf.map`; do not infer stack safety from SDK defaults,
which normally place stacks in 4KiB scratch banks. Check the final map and
artifact checksum after the last source commit/build.

Verified local integration gates: `make format-check`, `make lint-check`,
`make test`, `make build-pico`, and the additional `make build-pico2` all pass.
The test target reports 141 Unity tests, five unchanged startup/reporting
regressions, the real-Bosch SPI emulator, bounded USB buffer tests, and 28
runtime/parser/safety tests. All 28 runtime cases also pass with
`CONTROL_RUNTIME_SANITIZERS=address,undefined`. The coordinator separately reports
344 passing Python tests, including differential checks against actual C and
frozen original Python, at firmware commit `0e8903f`. The coordinator reports
that Python is staged on the Pi but no new image has been flashed. Those Python
checks and staging were not performed by the MCU safety implementation agent.

Generate a portable final manifest after building the committed source:
`bash scripts/control_artifact_manifest.sh > build/pico/artifact-manifest.txt`.
Copy that manifest with the UF2 and this handoff. Compiler `.su` stack reports
are retained under `build/pico/CMakeFiles/firmware.dir`; observed individual
frames include `send_frame` 1608 bytes, settings decode 688, and controller
step 712. These are per-function estimates, not measured whole-stack maxima.
The final map/manifest is authoritative if a later compiler changes layout.
A passing host test is not a real RP2040 duty or motor-timing measurement.

## Safety blocker verification (2026-09-15 UTC)

The permanent reproductions are in `tests/test_control_runtime.py` and
`tests/control_runtime_mocks/runtime_harness.c`. They compile the real runtime,
controller, parser and device USB queues against deterministic SDK boundaries.
The fake clock is genuinely 64-bit; wrap tests keep both core heartbeats current
so a simulated stall cannot hide a missing expiry latch. The original eight
wrap/stall regressions failed before the fixes and pass afterward.

| Case | Reproduction and corrected behavior |
| --- | --- |
| CONTROL lifetime | A command received at 1000 us is expired at 202000 us. After `2^32 + 1001` us the same command remains invalid. Core1 retires `command_valid`; core0 permanently inhibits expired authority and advances the accepted sequence floor. |
| RAW lifetime | RAW received at 1000 us expires at 201001 us. Advancing to `2^32 + 1001` us without another RAW packet cannot restore the old motor values. |
| Pressure lifetime | A healthy reading timestamped 1000 us expires at 501001 us. A fresh CONTROL after the clock alias cannot run depth PID against that cached reading. Only a fresh healthy PRESSURE restores pressure validity. |
| IMU lifetime | An initialized sensor returning NOT_READY with `sample_time=1000` becomes invalid at 11001 us and stays invalid after the clock alias. `sample_valid` is restored only by an actual fresh sample; AHRS restarts its sample dt at 2 ms after the gap. |
| First queued CONTROL | Core0 tracks accepted CONTROL ingress time separately from the latest output snapshot. A fresh first command, including one after an old lease expired, survives the neutral output check while queued and executes once when core1 gets a fresh sample. Old output is never made fresh by that newer command. |
| Neutral-session stall | A 500 ms core0/USB-worker stall is detected even with neutral motors and the physical-output timer disarmed. Tests cover CONTROL and RAW dispatch before IRQ/service, IRQ before dispatch, and service-only recovery. The old session is poisoned before dispatch or heartbeat overwrite; recovery requires a different HELLO session, settings and fresh CONTROL. |
| Uninitialized-output recovery | A stall between HELLO and protocol initialization cannot wait forever for a waveform service that main correctly skips. Service completes recovery when outputs are uninitialized; the old HELLO remains rejected and a different HELLO succeeds. With initialized outputs, a new HELLO remains BUSY until physical neutral is serviced. |
| COMMIT ACK ordering | At the actual USB write of APPLIED COMMIT bytes, `pending_commit` is false and the completed maintenance gate is already published to core1. An immediately following CONTROL executes without an extra core0 service pass. This closes an additional first-command race found during the safety audit. |
| Neutral protocol waits | A mocked protocol callback advances time by 20 ms and invokes the actual safety IRQ. Both success and rejection preserve the session during this deliberate inhibited wait; rejection refreshes the heartbeat before removing the exemption. |

`src/control/runtime.c` checks expiry before command processing, sample handling
and snapshot publication. Stall checks run before direct extended/legacy ingress,
before service refreshes the heartbeat, and in `src/main.c` before `usb_poll`.
The IRQ also checks active-session heartbeat age independently of nonneutral
output. Explicit inhibited maintenance remains exempt from the session-stall
check. Ordinary idle ESC-version discovery now queues ten EDT repeats through
the existing channel scheduler rather than invoking the blocking startup helper.
This matches the existing asynchronous `dshot_enable_edt_if_idle` pattern;
startup command sequencing itself is unchanged and its regression still passes.

Final independent gate run, all exit codes zero:

| Command | Evidence under `build/verification/` |
| --- | --- |
| `make format-check` | `format-check-safety-final.log` |
| `make test` | `test-safety-final.log`: 141 Unity, 5 startup/reporting, Bosch SPI, USB buffers, 28 runtime tests |
| `make lint-check` | `lint-check-safety-final.log` |
| `make build-pico` | `build-pico-safety-final.log` |
| `make build-pico2` | `build-pico2-safety-final.log` |
| `CONTROL_RUNTIME_SANITIZERS=address,undefined python3 tests/test_control_runtime.py` | `runtime-sanitized-safety-final.log`: 28 passed |

Each gate log has a matching `.exit` file. `git diff --check` also passes.
The commands above rerun without the old `/tmp` evidence directory. This section
retains the results and hashes even if ignored build logs are later removed.

Verified source SHA-256:

| File | SHA-256 |
| --- | --- |
| `src/control/runtime.c` | `8b7acdc6fa0f4fc07520e96913d8df451c0acc0cfcc5416c7221720941a33ab5` |
| `src/control/runtime.h` | `cd6210493b8b74f623675b37859abb8f2fee964b797aa20ba4cadfa2fe90b874` |
| `src/main.c` | `59912049eeedf7c30adf435bda03979ef0d96a075fcd76b744c01f8adad99760` |
| `tests/control_runtime_mocks/runtime_harness.c` | `1b53a8381199fa58683e9e674060dee36dccea99ea688e6147a9d9e45864b071` |
| `tests/test_control_runtime.py` | `eb843234f62e0829d1f89fa3b441c375ed28b69ad134116dacdc0c7f14ddbcb0` |

Verified working-tree build SHA-256:

| Artifact | SHA-256 |
| --- | --- |
| `build/pico/firmware.uf2` | `dd07510dea6ca63f49f2664c9f396ce55bbc297b507397e4f6e80ce7d8f7473d` |
| `build/pico/firmware.elf` | `5febfaa79388a8032f96ae834d60d01d46d9cdf4a8e2f1e9bbf96e9ebea954ef` |
| `build/pico2/firmware.uf2` | `408bf96e8404ede084340f9a15c03fe1241091660c73194408478fec38dd6ffa` |

Both builds embed `pico-control-dev:7f152f8f0473-dirty`. The marker alone cannot
identify the corrections; compare the artifact hash. Toolchain: Arm GNU
14.2.Rel1, GCC 14.2.1 (20241119), CMake 3.31.6. `arm-none-eabi-size` reports
Pico text/data/bss 113072/0/89484 bytes and Pico2 104448/0/88844 bytes. The Pico map
reserves core0 stack `[0x2003c000, 0x20040000)` with `__HeapLimit=0x2003c000`;
core1 retains its explicit 8192-byte BSS stack. Individual compiler frames are
1608 bytes for `send_frame`, 688 for settings decode, 712 for controller step,
and 256 for `core1_main`. These are not whole-stack high-water measurements.
The generated local manifest is `build/verification/artifact-safety-final.txt`.
Rebuild and regenerate hashes after any later source change or commit.

Scoped `.gitattributes` entries disable text conversion for the five pinned
Bosch C/header files and LICENSE; only those C/header files receive
`whitespace=cr-at-eol`. No upstream bytes changed. All six SHA-256 values still
match `third_party/bmi270/README.md`, including original line endings. The pure
controller math was not changed by this safety correction. No hardware access,
flashing, commits or remote Git operations were performed during this correction.

## Coordinator-only hardware procedure

The implementation agents must not run the commands in this section. The
coordinator owns hardware, backup, deployment and rollback. ESCs must remain
disconnected for this smoke test. Stop if the Pi is inaccessible.

1. Preserve the installed Pi source/config, service unit and environment, and
   installed Pico UF2 artifacts. Record hashes. A UF2 filename is not proof of
   the running binary. Keep credentials outside logs and repository artifacts.
2. Stop `manafish-firmware.service`; verify no service, terminal or other process
   owns `/dev/ttyACM0`. Confirm the USB device is the intended original RP2040
   Pico, not Pico 2 and not another board.
3. With ESCs disconnected, save the actual Pico flash before replacing it:
   `picotool save -a pico-before.uf2 -f`. This command deliberately enters the
   bootloader; record the result/hash. If a recoverable backup cannot be made,
   stop rather than assume the installed RC6-named file matches running flash.
4. Copy the verified new `build/pico/firmware.uf2` to the Pi/test host; compare
   SHA-256 on both ends. Use `picotool load -f firmware.uf2`, then
   `picotool reboot`. Wait for the CDC device and verify capability identity.
5. Run the isolated smoke test with the existing host pyserial environment:

   ```sh
   python3 scripts/control_smoke.py --port /dev/ttyACM0 --seconds 12 --execute
   ```

   This writes a RAM-only zero-allocation, zero-power, no-nullspace controller
   configuration, so controller targets/PID can run without nonneutral motors.
   Pressure in this isolated smoke is explicitly synthetic, not a Pi pressure
   sensor measurement. It verifies capability, atomic settings, invalid settings
   rejection/QUERY, quaternion/depth setters, raw IMU, statistics, host expiry
   and sticky neutral maintenance. It never programs an ESC.
6. Install the paired Python worktree separately from the preserved deployment,
   enable the explicit development override, and start only that isolated test
   service. Verify real Pi I2C pressure at 15Hz, source commands at 60Hz, and the
   full app calibration/maintenance paths with coordinator instrumentation.
   The service must re-enter through HELLO + settings ACK + fresh CONTROL.
7. Record measured five-second AHRS/PID/depth rates, min/average/max/p99-bucket
   execution duration, misses, sensor errors, queue overflow, output/host ages,
   USB drops, and each DShot motor's transmitted-frame rate. Repeat with maximum
   supported nullspace workload and host USB backpressure. With ESCs physically
   disconnected, opt into the dense eight-vector workload using
   `--stress-nullspace --escs-disconnected` on the smoke command. This mode CAN
   produce nonneutral calculated motor commands despite zero user power;
   never run it with connected ESCs. Estimate 1kHz duty
   from measured cost; do not claim an untested 1kHz loop.

   Add `--backpressure` to retain a five-second statistics window containing
   four seconds without host serial reads while fresh CONTROL and PRESSURE
   continue. The helper requires a checksum-valid increase in `USB_drops`,
   timely delivery of that specific window, 495–505 Hz AHRS/PID, no missed
   slots/queue overflow, and maximum execution below 2000 us. It fails rather
   than substituting a later, unblocked window as timing proof. Six offline
   `tests/test_control_smoke.py` checks cover this evidence handling and default
   neutral settings; they are included in `make test`, not hardware results.

The coordinator reports that the original full-flash UF2 and Pi source/config/
service backups are verified at `/home/pi/pico-spi-attitude-500hz/backup`, and
that the original D6 release response is `1.0.3-rc.6`. These are coordinator
observations, not hardware operations performed by the implementation agents.

Existing PWM remains at **50 Hz** (`src/pwm/pwm.h`); a 500 Hz computation loop
cannot make that waveform update at 500 Hz. No PWM timing change was made.
A 500Hz controller counter does not prove 500Hz accepted output on eight ESCs.
The two PIO groups now start together and share transmission-relative receive
deadlines, avoiding two sequential 500us receive waits. Each round still services
one channel per group; four rounds cover all motors. Real per-channel reception,
loaded behavior, ESC detection and programming require powered-ESC testing.

## Rollback and recovery

- Stop the test Pi service and every serial client first. Disconnect ESCs.
- Restore the saved `pico-before.uf2` with `picotool load -f`, then reboot. If CDC
  is unavailable, hold the Pico BOOTSEL button while reconnecting USB and load
  through the ROM bootloader. Do not substitute a Pico 2 artifact.
- Restore the paired original Pi source/config/service environment and remove
  the development override before restarting the original service. Restoring
  only one side can leave incompatible control/configuration behavior.
- Verify the old version/capability behavior and neutral startup. Restore actual
  operation only under coordinator approval.
- Failed ESC programming retains the existing recovery lockout; the control
  migration never treats ABORT or upload completion as permission to spin motors.
  Use the existing complete-image recovery transaction while maintenance remains
  latched. Do not clear recovery merely to make a protocol/config ACK succeed.

PWM starvation forces a neutral hardware pulse. DShot starvation suppresses
its signal until normal neutral frames can resume; without attached ESCs this
cannot certify the downstream ESC failsafe delay. Both remain explicit powered
hardware release gates. No release/publish/deploy authorization is implied by
this handoff.
