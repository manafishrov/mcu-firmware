#!/usr/bin/env python3
"""Exercise main.c's actual DShot startup against recording host stubs.

MCU_TEST_REVISION optionally selects a Git baseline. No hardware is accessed.
"""

import ctypes
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class StartupCommandsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        revision = os.environ.get("MCU_TEST_REVISION")
        if revision:
            main = subprocess.check_output(
                ["git", "-C", str(ROOT), "show", revision + ":src/main.c"], text=True
            )
        else:
            main = (ROOT / "src/main.c").read_text()
        start = main.index("static void init_dshot_protocol(")
        end = main.index("static void init_current_protocol(", start)
        startup = main[start:end]
        legacy = "bool persist_3d_mode" in startup
        call = "init_dshot_protocol(speed, true);" if legacy else "init_dshot_protocol(speed);"
        constants = "\n".join(re.findall(
            r"^#define (?:DSHOT_PIO|DSHOT_SM_[01]|DSHOT_TELEMETRY_WARNING_DELAY_MS) .+$",
            main, re.MULTILINE
        ))
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "dshot/control.h"
#include "dshot/current_sensing.h"
#include "dshot/telemetry_usb.h"
'''+constants+r'''
static struct dshot_controller dshot_controller0, dshot_controller1;
static dshot_telemetry_context_t dshot_context0, dshot_context1;
static bool edt_enable_scheduled[NUM_MOTORS], quality_warned[NUM_MOTORS];
static absolute_time_t edt_enable_time[NUM_MOTORS];
static uint16_t command_values[NUM_MOTORS];
static bool dshot_initialized, dshot_telemetry_warning_pending;
static absolute_time_t dshot_telemetry_warning_time, next_quality_report_time;
static int command_count, commands[16], repeats[16], initializations, registrations;
static int neutral_sends, frame_cycles, calibration_resets, telemetry_resets;
void dshot_controller_reset_calibration(void) { calibration_resets++; }
void dshot_telemetry_usb_init(void) { telemetry_resets++; }
void dshot_controller_init(struct dshot_controller* controller, uint16_t speed, PIO pio,
                           uint8_t sm, int pin, int channels) {
    assert(speed == 150 || speed == 300 || speed == 600);
    assert(pio == DSHOT_PIO);
    if (controller == &dshot_controller0) {
        assert(sm == DSHOT_SM_0 && pin == MOTOR0_PIN_BASE && channels == NUM_MOTORS_0);
    } else {
        assert(controller == &dshot_controller1);
        assert(sm == DSHOT_SM_1 && pin == MOTOR1_PIN_BASE && channels == NUM_MOTORS_1);
    }
    initializations++;
}
void dshot_telemetry_callback(void* context, int channel, enum dshot_telemetry_type type,
                              uint32_t value) {
    (void)context; (void)channel; (void)type; (void)value;
}
void dshot_register_telemetry_cb(struct dshot_controller* controller,
                                 dshot_telemetry_callback_t callback, void* context) {
    assert(callback == dshot_telemetry_callback);
    assert((controller == &dshot_controller0 && context == &dshot_context0)
        || (controller == &dshot_controller1 && context == &dshot_context1));
    registrations++;
}
void dshot_send_commands(uint16_t* values, struct dshot_controller* a,
                         struct dshot_controller* b) {
    assert(a == &dshot_controller0 && b == &dshot_controller1);
    for (int i = 0; i < NUM_MOTORS; i++) assert(values[i] == CMD_THROTTLE_NEUTRAL);
    neutral_sends++;
}
void dshot_run_frame_cycles(struct dshot_controller* a, struct dshot_controller* b, int cycles) {
    assert(a == &dshot_controller0 && b == &dshot_controller1);
    frame_cycles += cycles;
}
void dshot_send_command_to_all(struct dshot_controller* a, struct dshot_controller* b,
                               uint16_t command, uint8_t repeat_count) {
    assert(a == &dshot_controller0 && b == &dshot_controller1);
    assert(command_count < 16);
    commands[command_count] = command;
    repeats[command_count++] = repeat_count;
}
'''+startup+r'''
void run_startup(uint16_t speed) {
    (void)pio1; // The shared mock header also defines this unused controller.
    command_count = initializations = registrations = neutral_sends = frame_cycles = 0;
    calibration_resets = telemetry_resets = 0;
    dshot_initialized = dshot_telemetry_warning_pending = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        command_values[i] = CMD_THROTTLE_NEUTRAL;
        edt_enable_scheduled[i] = quality_warned[i] = true;
    }
'''+call+r'''
    assert(initializations == 2 && registrations == 2);
    assert(neutral_sends == 1 && frame_cycles == NUM_MOTORS * 4);
    assert(calibration_resets == 1 && telemetry_resets == 1);
    assert(dshot_initialized && dshot_telemetry_warning_pending);
    assert(next_quality_report_time == dshot_telemetry_warning_time);
    for (int i = 0; i < NUM_MOTORS; i++)
        assert(!edt_enable_scheduled[i] && !quality_warned[i]);
}
int sent_count(void) { return command_count; }
int sent_command(int i) { return commands[i]; }
int sent_repeats(int i) { return repeats[i]; }
int telemetry_enable_command(void) { return DSHOT_EXTENDED_TELEMETRY_ENABLE; }
'''
        cls.temp = tempfile.TemporaryDirectory(prefix="pico-startup-")
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        source = folder / "startup.c"
        library = folder / "startup.so"
        source.write_text(harness)
        subprocess.run(
            shlex.split(os.environ.get("CC", "cc"))
            + ["-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
               "-I", str(ROOT / "tests/mocks"), "-I", str(ROOT / "src"),
               str(source), str(ROOT / "src/dshot/current_sensing.c"),
               "-o", str(library)], check=True
        )
        cls.dll = ctypes.CDLL(str(library))
        cls.dll.run_startup.argtypes = [ctypes.c_uint16]
        cls.dll.run_startup.restype = None

    def test_startup_only_sends_volatile_telemetry_enable(self):
        for speed in (150, 300, 600):
            with self.subTest(speed=speed):
                self.dll.run_startup(speed)
                self.assertEqual(self.dll.sent_count(), 1)
                self.assertEqual(self.dll.sent_command(0), self.dll.telemetry_enable_command())
                self.assertEqual(self.dll.sent_repeats(0), 10)

    def test_reinitialization_does_not_write_settings(self):
        for _ in range(3):
            self.dll.run_startup(300)
            self.assertEqual(self.dll.sent_count(), 1)
            self.assertEqual(self.dll.sent_command(0), self.dll.telemetry_enable_command())


class CurrentReportingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        main = (ROOT / "src/main.c").read_text()
        start = main.index("static void service_current_reporting(")
        service = main[start:main.index("int main(void)", start)]
        harness = r'''
#include <stdbool.h>
#include <stdint.h>
#include "dshot/current_sensing.h"
#include "dshot/telemetry_usb.h"
#define THRUSTER_PROTOCOL_DSHOT 1
static bool runtime_config_received = true, protocol_initialized = true;
static bool esc_firmware_recovery_mode, uploading, seen_enabled;
static struct { int protocol; } current_config = {THRUSTER_PROTOCOL_DSHOT};
static uint16_t command_values[NUM_MOTORS];
static uint32_t now_ms;
static int reports, flushes, service_calls;
uint32_t get_absolute_time(void) { return now_ms; }
uint32_t to_ms_since_boot(uint32_t value) { return value; }
bool esc_firmware_update_receiving(void) { return uploading; }
void current_sensing_service(const uint16_t values[NUM_MOTORS], bool enabled, uint32_t now) {
    (void)values; (void)now; seen_enabled = enabled; service_calls++;
}
int32_t current_sensing_current_ma(uint8_t board, uint32_t now) {
    (void)board; (void)now; return 1000;
}
int32_t current_sensing_baseline_ma(uint8_t board) { (void)board; return 91000; }
void dshot_telemetry_usb_send(uint8_t motor, uint8_t type, int32_t value) {
    (void)motor; (void)type; (void)value; reports++;
}
void dshot_telemetry_usb_flush(void) { flushes++; }
''' + service + r'''
void run_reporting(int upload, int recovery) {
    (void)pio0; (void)pio1;
    uploading = upload != 0; esc_firmware_recovery_mode = recovery != 0;
    now_ms += 1000; reports = flushes = service_calls = 0;
    service_current_reporting();
}
int report_count(void) { return reports; }
int flush_count(void) { return flushes; }
int sensing_enabled(void) { return seen_enabled; }
int sensing_calls(void) { return service_calls; }
'''
        cls.temp = tempfile.TemporaryDirectory(prefix="pico-current-reporting-")
        cls.addClassCleanup(cls.temp.cleanup)
        source = Path(cls.temp.name) / "reporting.c"
        library = source.with_suffix(".so")
        source.write_text(harness)
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
            "-I", str(ROOT / "tests/mocks"), "-I", str(ROOT / "src"),
            str(source), "-o", str(library)
        ], check=True)
        cls.dll = ctypes.CDLL(str(library))
        cls.dll.run_reporting.argtypes = [ctypes.c_int, ctypes.c_int]
        cls.dll.run_reporting.restype = None

    def test_upload_and_recovery_disable_sensing_and_usb_reports(self):
        for upload, recovery in ((1, 0), (0, 1), (1, 1)):
            self.dll.run_reporting(upload, recovery)
            self.assertEqual(self.dll.sensing_calls(), 1)
            self.assertEqual(self.dll.sensing_enabled(), 0)
            self.assertEqual(self.dll.report_count(), 0)
            self.assertEqual(self.dll.flush_count(), 0)

    def test_normal_operation_resumes_reporting(self):
        self.dll.run_reporting(1, 0)
        self.dll.run_reporting(0, 0)
        self.assertEqual(self.dll.sensing_enabled(), 1)
        self.assertEqual(self.dll.report_count(), 4)
        self.assertEqual(self.dll.flush_count(), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
