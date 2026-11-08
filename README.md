# Manafish MCU Firmware

Thruster firmware for the Raspberry Pi Pico and Pico 2 in the Manafish ROV. It
drives the ESCs with DShot or PWM, selected at runtime, and talks to the ROV
firmware over USB.

## Development

`nix develop` (or `direnv allow`) provides the toolchain. CMake downloads the
Pico SDK.

```sh
make build-pico   # or build-pico2
make flash-pico   # or flash-pico2
make test
```

`make help` lists every target. Debug output is on the USB serial port at
115200 baud, for example `picocom -b 115200 /dev/ttyACM0`.

## License

[AGPL-3.0-or-later](LICENSE)
