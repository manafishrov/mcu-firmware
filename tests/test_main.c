#include "esc_firmware/update.h"
#include "unity/unity.h"

extern void test_usb_comm(void);
extern void test_runtime_config(void);
extern void test_dshot_control(void);
extern void test_dshot_protocol(void);
extern void test_pwm_control(void);
extern void test_pwm_driver(void);
extern void test_esc_firmware_update(void);
extern void test_esc_version_telemetry(void);

void setUp(void) {
    esc_firmware_update_reset();
}

void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();
    test_usb_comm();
    test_runtime_config();
    test_dshot_control();
    test_dshot_protocol();
    test_pwm_control();
    test_pwm_driver();
    test_esc_firmware_update();
    test_esc_version_telemetry();
    return UNITY_END();
}
