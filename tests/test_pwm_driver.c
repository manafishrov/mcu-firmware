#include "pwm/pwm.h"
#include "unity/unity.h"
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <string.h>

int mock_gpio_function[30];
int mock_gpio_direction[30];
int mock_gpio_value[30];
bool mock_pwm_enabled[8];
float mock_pwm_divider[8];
uint16_t mock_pwm_wrap[8];
uint16_t mock_pwm_level[30];
uint16_t mock_pwm_counter[8];

static void reset_pwm_mocks(void) {
    memset(mock_gpio_function, 0, sizeof(mock_gpio_function));
    memset(mock_gpio_direction, 0, sizeof(mock_gpio_direction));
    memset(mock_gpio_value, 0, sizeof(mock_gpio_value));
    memset(mock_pwm_enabled, 0, sizeof(mock_pwm_enabled));
    memset(mock_pwm_divider, 0, sizeof(mock_pwm_divider));
    memset(mock_pwm_wrap, 0, sizeof(mock_pwm_wrap));
    memset(mock_pwm_level, 0, sizeof(mock_pwm_level));
    memset(mock_pwm_counter, 0, sizeof(mock_pwm_counter));
}

static void test_pwm_init_configures_all_eight_motor_outputs_before_enable(void) {
    reset_pwm_mocks();
    struct pwm_controller controller;
    uint pins[PWM_MAX_CHANNELS] = {6, 7, 8, 9, 18, 19, 20, 21};

    pwm_controller_init(&controller, pins, PWM_MAX_CHANNELS, 1500);

    TEST_ASSERT_EQUAL_UINT(PWM_MAX_CHANNELS, controller.num_channels);
    for (uint channel = 0; channel < PWM_MAX_CHANNELS; ++channel) {
        TEST_ASSERT_EQUAL_INT(GPIO_FUNC_PWM, mock_gpio_function[pins[channel]]);
        TEST_ASSERT_EQUAL_UINT16(1500, mock_pwm_level[pins[channel]]);
    }
    for (uint slice = 1; slice <= 4; ++slice) {
        TEST_ASSERT_TRUE(mock_pwm_enabled[slice]);
        TEST_ASSERT_EQUAL_UINT16(PWM_WRAP, mock_pwm_wrap[slice]);
        TEST_ASSERT_EQUAL_UINT16(0, mock_pwm_counter[slice]);
    }
}

static void test_pwm_deinit_disables_every_slice_and_drives_outputs_low(void) {
    reset_pwm_mocks();
    struct pwm_controller controller;
    uint pins[PWM_MAX_CHANNELS] = {6, 7, 8, 9, 18, 19, 20, 21};
    pwm_controller_init(&controller, pins, PWM_MAX_CHANNELS, 1500);

    pwm_controller_deinit(&controller);

    TEST_ASSERT_EQUAL_UINT(0, controller.num_channels);
    for (uint channel = 0; channel < PWM_MAX_CHANNELS; ++channel) {
        TEST_ASSERT_EQUAL_INT(GPIO_FUNC_SIO, mock_gpio_function[pins[channel]]);
        TEST_ASSERT_EQUAL_INT(GPIO_OUT, mock_gpio_direction[pins[channel]]);
        TEST_ASSERT_EQUAL_INT(0, mock_gpio_value[pins[channel]]);
    }
    for (uint slice = 1; slice <= 4; ++slice) {
        TEST_ASSERT_FALSE(mock_pwm_enabled[slice]);
    }
}

void test_pwm_driver(void) {
    RUN_TEST(test_pwm_init_configures_all_eight_motor_outputs_before_enable);
    RUN_TEST(test_pwm_deinit_disables_every_slice_and_drives_outputs_low);
}
