# Pico SPI control handoff

Task: `pico-spi-attitude-500hz`. Baseline MCU `47c503e`; branch
`feat/pico-spi-attitude-500hz`. This is a development build, not a release.
No version bump, push, PR, tag, release or hardware operation was performed by
the MCU implementation agents.

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
regressions, the real-Bosch SPI emulator, bounded USB buffer tests, and 15
compiled runtime/parser/safety tests. The 15 runtime cases also pass with
`CONTROL_RUNTIME_SANITIZERS=address,undefined`. The paired Python agent reports
339 passing tests including 25 differential checks against actual C and frozen
original Python; see that repository's own final verification for subsequent
host changes.

Generate a portable final manifest after building the committed source:
`bash scripts/control_artifact_manifest.sh > build/pico/artifact-manifest.txt`.
Copy that manifest with the UF2 and this handoff. Compiler `.su` stack reports
are retained under `build/pico/CMakeFiles/firmware.dir`; observed individual
frames include `send_frame` 1608 bytes, settings decode 688, and controller
step 712. These are per-function estimates, not measured whole-stack maxima.
The final map/manifest is authoritative if a later compiler changes layout.
A passing host test is not a real RP2040 duty or motor-timing measurement.

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

The coordinator reports that the original full-flash UF2 and Pi source/config/
service backups are verified at `/home/pi/pico-spi-attitude-500hz/backup`, and
that the original D6 release response is `1.0.3-rc.6`. These are coordinator
observations, not hardware operations performed by the implementation agents.

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
