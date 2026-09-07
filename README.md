# Microcontroller Firmware

Firmware for the Raspberry Pi Pico and Pico 2 used in the Manafish ROV to
control thrusters. It supports two runtime-selectable control protocols:

- DShot (digital ESC control)
- PWM (analog ESC control)

## ESC input configuration

The Manafish AM60 V2 ESC firmware owns persistent input settings. This Pico
firmware no longer sends automatic 3D-mode or Save Settings commands on startup,
protocol changes, or recovery after an ESC update. Other ESC firmware must be
configured for compatible bidirectional operation before use.

Extended DShot Telemetry enable and idle retries remain: they are volatile
session handshakes, not EEPROM writes. Neutral output, protocol-switch quiet
intervals, version discovery, and failed-update recovery restrictions are
unchanged.

Install the AM32 image that enforces the required input settings on all eight
ESC controllers **before** installing this Pico image. The older Pico can
perform that ESC update. A simultaneous Pi bundle update does not enforce the
order: Pi firmware may auto-update the Pico before the operator flashes ESCs.
Stage the rollout accordingly. No USB format or app topology-setting change is
required.

These changes remove a demonstrated saved-settings failure path; they do not
prove that PWM detection/arming works on every installed ESC. Validate PWM
and DShot transitions on hardware before relying on the change.

## Prerequisites

- Raspberry Pi Pico SDK (automatically fetched by CMake)
- clang-format and clang-tidy
- picotool for flashing
- arm-none-eabi-gcc toolchain
- CMake and Make
- picocom for debugging

If you have **[Nix](https://nixos.org/)** and **[direnv](https://direnv.net/)** installed:

1. Enter the directory: `cd mcu-firmware`
2. Run `direnv allow`

This will automatically download and configure the Pico SDK, ARM toolchain,
Clang tools, and CMake.

## Building and Flashing

The project uses CMake to configure the build. The Pico SDK is downloaded
automatically on first build.

### Commands

Run `make help` to list all available targets.

Key targets include:

- `make build-pico` – Build unified thruster firmware for Pico
- `make build-pico2` – Build unified thruster firmware for Pico 2
- `make flash-pico` – Build and flash unified firmware for Pico
- `make flash-pico2` – Build and flash unified firmware for Pico 2
- `make clean` – Remove build directories
- `make format` – Format source code
- `make format-check` – Verify formatting (useful for CI)
- `make lint` – Lint and auto-fix C code
- `make lint-check` – Check C code lint
- `make test` – Run Unity tests and the Python/C startup-command regression

### Build Output

Compiled `.uf2` files appear in:

- `build/pico/firmware.uf2` / `build/pico2/firmware.uf2`

### Flashing

Use `make flash-pico` or `make flash-pico2` for the unified firmware.
Flashing works regardless of whether the Pico is in BOOTSEL mode—the device
reboots automatically as needed.

## Debugging

The firmware outputs debug messages via USB CDC. Use a serial monitor
like `picocom` to debug:

1. Find the Pico's serial port:

   ```sh
   ls /dev/ttyACM*  # Linux
   ls /dev/tty.usbmodem*  # Darwin
   ```

2. Connect to the device:

   ```sh
   # Linux
   picocom -b 115200 /dev/ttyACM0

   # Darwin
   picocom -b 115200 /dev/tty.usbmodem*
   ```

3. Exit with **Ctrl+A**, then **K** and confirm.

## Development Hooks

Install the Git hook once per clone:

```sh
pre-commit install
```

The pre-commit hook runs clang-format on committed C/C++ files before each
commit. To run the same checks across the repository manually:

```sh
pre-commit run --all-files
```

To update hook versions later:

```sh
pre-commit autoupdate
```

## License

This project is licensed under the GNU Affero General Public License
v3.0 or later - see the [LICENSE](LICENSE) file for details.
