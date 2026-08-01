#ifndef MOCK_HARDWARE_GPIO_H
#define MOCK_HARDWARE_GPIO_H

#include "../mock_sdk.h"

#define GPIO_FUNC_SIO 0
#define GPIO_FUNC_PWM 4
#define GPIO_OUT 1

typedef uint mock_gpio_uint_t;

extern int mock_gpio_function[30];
extern int mock_gpio_direction[30];
extern int mock_gpio_value[30];

static inline void gpio_set_pulls(uint pin, bool up, bool down) {
    (void)pin;
    (void)up;
    (void)down;
}

static inline void gpio_set_function(uint pin, int fn) {
    mock_gpio_function[pin] = fn;
}

static inline void gpio_disable_pulls(uint pin) {
    (void)pin;
}

static inline void gpio_set_dir(uint pin, int out) {
    mock_gpio_direction[pin] = out;
}

static inline void gpio_put(uint pin, int value) {
    mock_gpio_value[pin] = value;
}

#endif
