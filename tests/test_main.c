#include "unity/unity.h"

extern void test_usb_comm(void);
extern void test_runtime_config(void);
extern void test_dshot_control(void);
extern void test_dshot_protocol(void);
extern void test_pwm_control(void);
extern void test_pwm_driver(void);

void setUp(void) {}

void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();
    test_usb_comm();
    test_runtime_config();
    test_dshot_control();
    test_dshot_protocol();
    test_pwm_control();
    test_pwm_driver();
    return UNITY_END();
}
