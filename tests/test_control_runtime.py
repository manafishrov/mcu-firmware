#!/usr/bin/env python3
"""Run the real control runtime, parser and device USB queues without hardware.

Each scenario starts a fresh executable, preserving C static initialization.
Only Pico/TinyUSB/BMI270 hardware boundaries are mocked. Controller calls are
counted and forwarded to the actual pure implementation. No core1 thread runs;
tests interleave the production core1 helpers and core0 service deterministically.

Run: python3 tests/test_control_runtime.py
Optional: CONTROL_RUNTIME_SANITIZERS=address,undefined (GCC/Clang host toolchain).
"""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MOCKS = ROOT / "tests/control_runtime_mocks"
SCENARIOS = (
    "staging",
    "empty_abort_idempotent",
    "bad_settings_and_expiry",
    "queued_commit_timeout",
    "duplicate_fingerprint",
    "gate_retry",
    "irq_starvation",
    "pwm_and_host_irq",
    "commands_pressure_and_sensor",
    "stale_hol_commit",
    "sensor_retry_handshake",
    "parser_atomic_and_timeout_tail",
    "rx_arrival_lease_and_overflow",
    "rx_old_extended_lease",
)


class ControlRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="pico-control-runtime-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.executable = Path(cls.temp.name) / "runtime-tests"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-UNDEBUG",
            "-DPICO_ON_DEVICE=1", "-DPICO_RP2350=0",
            "-I", str(MOCKS), "-I", str(ROOT / "src"),
            str(MOCKS / "runtime_harness.c"),
            str(ROOT / "src/control/controller.c"),
            str(ROOT / "src/control/protocol.c"),
            str(ROOT / "src/pwm/control.c"),
            str(ROOT / "src/usb_comm.c"),
            str(ROOT / "src/usb_rx.c"),
            str(ROOT / "src/usb_tx.c"),
            "-lm", "-o", str(cls.executable),
        ]
        sanitizers = os.environ.get("CONTROL_RUNTIME_SANITIZERS")
        if sanitizers:
            command += [f"-fsanitize={sanitizers}", "-fno-omit-frame-pointer", "-g"]
            if os.uname().sysname == "Linux":
                command += ["-no-pie"]
        subprocess.run(command, check=True, text=True, timeout=60)

    def run_scenario(self, scenario):
        result = subprocess.run(
            [str(self.executable), scenario], capture_output=True, text=True, timeout=15
        )
        self.assertEqual(
            result.returncode, 0,
            f"{scenario}: exit {result.returncode}\n{result.stdout}\n{result.stderr}",
        )
        self.assertIn(f"PASS {scenario}", result.stdout)

    def test_main_keeps_negotiated_and_maintenance_legacy_gates(self):
        # This is explicitly a source wiring check, not a simulated main loop.
        # Runtime gate behavior is exercised by the compiled parser scenarios.
        source = (ROOT / "src/main.c").read_text()
        command_start = source.index("static void handle_command_packet(")
        config_start = source.index("static void handle_config_packet(", command_start)
        command = source[command_start:config_start]
        self.assertRegex(
            command,
            r"control_runtime_extended_active\(\)\s*\|\|\s*"
            r"control_runtime_maintenance_latched\(\)",
        )
        self.assertIn("!all_commands_neutral()", command)
        self.assertIn("control_runtime_inhibit();", command)
        self.assertIn("control_runtime_legacy_input_at(command_values, received_us)", command)
        config_end = source.index("\nstatic ", config_start + 1)
        config = source[config_start:config_end]
        self.assertRegex(
            config,
            r"control_runtime_extended_active\(\)\s*\|\|\s*"
            r"control_runtime_maintenance_latched\(\)",
        )
        self.assertRegex(source, r"control_runtime_receive_at\(control_buf,")


def scenario_test(name):
    def test(self):
        self.run_scenario(name)
    test.__name__ = f"test_{name}"
    return test


for _scenario in SCENARIOS:
    setattr(ControlRuntimeTests, f"test_{_scenario}", scenario_test(_scenario))

if __name__ == "__main__":
    unittest.main(verbosity=2)
