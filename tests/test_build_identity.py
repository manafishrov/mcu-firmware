#!/usr/bin/env python3
"""Evaluate production CMake identity wiring and decode real host-built USB replies.

CMake script mode evaluates the actual metadata/compile-definition blocks without
fetching the Pico SDK. The runtime harness supplies hardware boundaries only.
ARM builds separately verify the generated definition and embedded CAPS string.
"""

import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BuildIdentityTests(unittest.TestCase):
    def check_identity(self, release):
        with tempfile.TemporaryDirectory(prefix="mcu-build-identity-") as directory:
            temp = Path(directory)
            cmake = (ROOT / "CMakeLists.txt").read_text()
            metadata = cmake[
                cmake.index("set(FIRMWARE_EXE_NAME") : cmake.index("\nconfigure_file(")
            ]
            definitions = re.search(
                r"target_compile_definitions\([\s\S]*?\n\)", cmake
            ).group(0)
            script = temp / "identity.cmake"
            script.write_text(
                'function(target_compile_definitions)\n'
                '  foreach(definition IN LISTS ARGN)\n'
                '    if(definition MATCHES "^CONTROL_BUILD_IDENTITY=")\n'
                f'      file(APPEND "{temp.as_posix()}/definition.txt" "${{definition}}\\n")\n'
                '    endif()\n'
                '  endforeach()\n'
                'endfunction()\n' + metadata + '\n' + definitions + '\n'
                f'configure_file("{ROOT.as_posix()}/src/release_version.h.in" '
                f'"{temp.as_posix()}/release_version.h" @ONLY)\n'
            )
            subprocess.run(
                ["cmake", f"-DMANAFISH_RELEASE_VERSION={release}", "-P", str(script)],
                cwd=ROOT, check=True, timeout=30,
            )
            definition = (temp / "definition.txt").read_text().strip()
            if release:
                expected = release
            else:
                commit = subprocess.check_output(
                    ["git", "rev-parse", "--short=12", "HEAD"], cwd=ROOT, text=True
                ).strip()
                dirty = subprocess.check_output(
                    ["git", "status", "--porcelain"], cwd=ROOT, text=True
                ).strip()
                expected = f"pico-control-dev:{commit}" + ("-dirty" if dirty else "")
            self.assertEqual(definition, f'CONTROL_BUILD_IDENTITY="{expected}"')
            legacy = release or re.search(
                r'#define MANAFISH_RELEASE_VERSION "([^"]+)"',
                (ROOT / "src/version.h").read_text(),
            ).group(1)
            source = temp / "identity.c"
            source.write_text(
                '#define main runtime_harness_main\n'
                f'#include "{ROOT.as_posix()}/tests/control_runtime_mocks/runtime_harness.c"\n'
                '#undef main\n#include "runtime_config.h"\n'
                'int main(int argc, char **argv) {\n'
                '  assert(argc == 3);\n'
                '  init_runtime(); control_runtime_capabilities(7); flush_tx();\n'
                '  control_frame_t frame;\n'
                '  assert(control_frame_decode(sent, sent_length, &frame));\n'
                '  assert(frame.type == FRAME_CAPS && frame.sequence == 7);\n'
                '  assert(frame.length == 12 + strlen(argv[1]));\n'
                '  assert(memcmp(frame.payload + 12, argv[1], strlen(argv[1])) == 0);\n'
                '  uint8_t packet[128];\n'
                '  size_t length = mcu_runtime_config_build_release_packet(packet, sizeof(packet), 7);\n'
                '  assert(length == strlen(argv[2]) + USB_RELEASE_VERSION_PACKET_OVERHEAD);\n'
                '  assert(packet[2] == strlen(argv[2]));\n'
                '  assert(memcmp(packet + 3, argv[2], strlen(argv[2])) == 0);\n'
                '  printf("CAPS=%s legacy=%s\\n", argv[1], argv[2]);\n'
                '  return 0;\n}\n'
            )
            executable = temp / "identity"
            command = shlex.split(os.environ.get("CC", "cc")) + [
                "-std=c11", "-Wall", "-Wextra", "-Werror", "-UNDEBUG",
                "-DPICO_ON_DEVICE=1", "-DPICO_RP2350=0", f"-D{definition}",
                "-I", str(temp), "-I", str(ROOT / "tests/control_runtime_mocks"),
                "-I", str(ROOT / "src"), str(source),
            ] + [str(ROOT / path) for path in (
                "src/control/controller.c", "src/control/protocol.c", "src/pwm/control.c",
                "src/runtime_config.c", "src/usb_comm.c", "src/usb_rx.c", "src/usb_tx.c",
            )] + ["-lm", "-o", str(executable)]
            subprocess.run(command, check=True, timeout=60)
            subprocess.run([str(executable), expected, legacy], check=True, timeout=15)

    def test_release_caps_uses_explicit_version(self):
        self.check_identity("1.0.4-rc.1")

    def test_development_caps_retains_git_identity(self):
        self.check_identity("")


if __name__ == "__main__":
    unittest.main(verbosity=2)
