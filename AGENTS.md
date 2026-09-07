# AGENTS.md

## Purpose

Thruster firmware for the Raspberry Pi Pico / Pico 2 used by the Manafish
ROV. Single binary supports two runtime-selectable ESC protocols: DShot
(digital) and PWM (analog). Communicates with the host firmware over USB CDC.

## Stack

- C11, Pico SDK (fetched by CMake)
- arm-none-eabi-gcc, CMake, Make, picotool, picocom
- clang-format, clang-tidy
- Unity for host-side unit tests
- Nix flake + direnv for the dev shell

## Structure

- `src/` — firmware sources (`main.c`, `usb_comm.*`, `runtime_config.*`,
  `log.*`, `dshot/`, `pwm/`)
- `tests/` — Unity tests (`test_*.c`), `mocks/`, `stubs/`, `support/`,
  `unity/`, and `test_startup_commands.py` (compiles the real startup function
  with recording C stubs)
- `CMakeLists.txt`, `pico_sdk_import.cmake` — build setup
- `Makefile` — wraps CMake for the common targets
- `flake.nix` — toolchain (Pico SDK, ARM GCC, Clang, CMake)

## Commands

Use the dev shell (`direnv allow` in repo, or `nix develop`). Then:

- Help: `make help`
- Build: `make build-pico` / `make build-pico2`
- Flash: `make flash-pico` / `make flash-pico2`
- Clean: `make clean`
- Debug serial: `picocom -b 115200 /dev/ttyACM0` (Linux) or
  `/dev/tty.usbmodem*` (Darwin)

### Quality (must pass before flashing or merging)

```sh
make format-check
make lint-check          # clang-tidy with -Werror
make test                # Unity host tests and Python/C startup regression
```

Auto-fix: `make format`, `make lint`.

Pre-commit hook runs `clang-format` on staged files. Install once:
`pre-commit install`.

## Rules

- Keep DShot and PWM behind the runtime selector in `runtime_config.*`. Don't
  fork the codebase per protocol.
- USB protocol lives in `usb_comm.*`. Changes here must be reflected in
  whatever host (firmware/app) consumes it.
- Match existing C style; no warnings in `lint-check`.
- Don't widen the toolchain (extra deps, alternative SDKs) without reason.
- Don't push without being asked. CI builds both Pico and Pico 2 artifacts.

## Releases

**Never cut a release without being explicitly asked, and confirm the version
and release notes back to the user before doing anything.**

- Bump the macros in `src/version.h`:
  `MCU_FIRMWARE_VERSION_MAJOR/MINOR/PATCH` to match `vX.Y.Z`.
- Commit message: `chore(release): vX.Y.Z`.
- Tag: `git tag vX.Y.Z` then `git push --tags` — pushing the tag triggers
  `.github/workflows/build.yml`, which builds Pico and Pico 2 `.uf2`
  artifacts and creates a **draft** GitHub release with auto-generated
  notes. Edit and publish manually.
- Pre-releases use `vX.Y.Z-rc.N` and are auto-marked prerelease.
- Quality gates above must pass first.

Workflow before tagging: confirm the bumped version with the user, confirm
the release notes text, then commit, tag, push.

## Commits

Conventional Commits, focused on **why**.

```
<type>(<scope>): <subject>

[body explaining why, ~72 char wrap]
```

- Types: `feat`, `fix`, `refactor`, `perf`, `docs`, `chore`, `ci`, `build`,
  `revert`. `chore(deps)` reserved for Renovate.
- Scopes: `dshot`, `pwm`, `usb`, `config`, `log`, `tests`, `cmake`, `flake`,
  `ci`.
- Subject: imperative, lowercase, ≤72 chars, no period.

## Keep this file useful

If you add a top-level source dir, change Makefile targets, swap toolchain
pieces, or alter the quality gates — update this file in the same commit.
