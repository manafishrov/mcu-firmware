"""Offline checks of bench-helper evidence handling; never opens hardware."""

import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
SPEC = importlib.util.spec_from_file_location(
    "control_smoke", Path(__file__).resolve().parents[1] / "scripts/control_smoke.py"
)
smoke = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(smoke)


class Port:
    def __init__(self, data):
        self.data = data

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, count):
        data, self.data = self.data[:count], self.data[count:]
        return data


class SmokeTests(unittest.TestCase):
    def test_default_settings_are_computationally_neutral(self):
        data = smoke.neutral_settings()
        self.assertEqual(len(data), 628)
        self.assertEqual(struct.unpack_from("<3f", data, 68), (0, 0, 0))
        self.assertEqual(struct.unpack_from("<64f", data, 92), (0,) * 64)
        self.assertEqual(struct.unpack_from("<I", data, 364), (0,))

    def test_only_checksum_valid_log_can_prove_usb_drops(self):
        message = b"Pico safety: USB_drops=123; duty=10.0%"
        packet = smoke.legacy(0xB5, bytes([1, len(message)]) + message)
        for corrupt in (False, True):
            with self.subTest(corrupt=corrupt):
                data = packet[:-1] + bytes([packet[-1] ^ int(corrupt)])
                link = smoke.Link(Port(data))
                with contextlib.redirect_stdout(io.StringIO()):
                    link.pump()
                self.assertEqual(link.usb_drops, None if corrupt else 123)
                self.assertEqual(link.counts["bad_legacy_checksum"], int(corrupt))

    def run_window(self, *, drops=True, late=False, misses=0):
        link = smoke.Link(Port(b""))
        link.usb_drops = 3
        clock = [0.0]
        calls = []

        def stream(_link, seconds, read=True, stop=None):
            calls.append((seconds, read))
            if not read:
                clock[0] += seconds
                return
            clock[0] = 5.0 if len(calls) == 1 else (15.0 if late else 10.0)
            link.stats.append({
                "received_host_monotonic_s": clock[0], "elapsed_s": 5.0,
                "ahrs_hz": 500, "pid_hz": 500, "misses": misses,
                "queue_overflows": 0, "max_us": 800,
            })
            if len(calls) > 1 and drops:
                link.usb_drops = 123

        with patch.object(smoke, "stream_window", side_effect=stream), \
                patch.object(smoke.time, "monotonic", side_effect=lambda: clock[0]), \
                contextlib.redirect_stdout(io.StringIO()):
            result = smoke.test_backpressure(link)
        self.assertEqual(calls[1], (4, False))
        return result

    def test_retains_blocked_window_and_requires_real_drop_delta(self):
        result = self.run_window()
        self.assertEqual(result["blocked_host_read_seconds"], 4)
        self.assertEqual(result["usb_drops_after"], 123)

    def test_no_drop_delta_is_not_backpressure_proof(self):
        with self.assertRaisesRegex(RuntimeError, "no proven USB backpressure"):
            self.run_window(drops=False)

    def test_later_unblocked_window_is_not_timing_proof(self):
        with self.assertRaisesRegex(RuntimeError, "Blocked statistics window was lost"):
            self.run_window(late=True)

    def test_deadline_misses_fail_acceptance(self):
        with self.assertRaisesRegex(RuntimeError, "deadline or queue failed"):
            self.run_window(misses=1)


if __name__ == "__main__":
    unittest.main()
