#ifndef MOCK_HARDWARE_PWM_H
#define MOCK_HARDWARE_PWM_H

#include "../mock_sdk.h"
#include <stdbool.h>

typedef uint mock_pwm_uint_t;

extern bool mock_pwm_enabled[8];
extern float mock_pwm_divider[8];
extern uint16_t mock_pwm_wrap[8];
extern uint16_t mock_pwm_level[30];
extern uint16_t mock_pwm_counter[8];

static inline uint pwm_gpio_to_slice_num(uint pin) {
    return (pin >> 1U) & 7U;
}

static inline void pwm_set_enabled(uint slice, bool enabled) {
    mock_pwm_enabled[slice] = enabled;
}

static inline void pwm_set_clkdiv(uint slice, float divider) {
    mock_pwm_divider[slice] = divider;
}

static inline void pwm_set_wrap(uint slice, uint16_t wrap) {
    mock_pwm_wrap[slice] = wrap;
}

static inline void pwm_set_gpio_level(uint pin, uint16_t level) {
    mock_pwm_level[pin] = level;
}

static inline void pwm_set_counter(uint slice, uint16_t counter) {
    mock_pwm_counter[slice] = counter;
}

#endif
