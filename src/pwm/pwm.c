#include "pwm.h"
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/platform_defs.h>
#include <hardware/pwm.h>
#include <hardware/structs/clocks.h>
#include <hardware/structs/io_bank0.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

void pwm_controller_init(struct pwm_controller *controller, uint *pins, uint num_channels,
                         uint initial_value) {
    memset(controller, 0, sizeof(*controller));
    if (num_channels > PWM_MAX_CHANNELS) {
        num_channels = PWM_MAX_CHANNELS;
    }
    controller->num_channels = num_channels;

    if (initial_value > PWM_WRAP) {
        initial_value = PWM_WRAP;
    }

    uint32_t clock_hz = clock_get_hz(clk_sys);
    float divider = (float)clock_hz / (float)(PWM_FREQUENCY * PWM_STEPS);
    bool slice_configured[NUM_PWM_SLICES] = {false};

    for (uint i = 0; i < num_channels; ++i) {
        controller->pin[i] = pins[i];
        gpio_set_function(pins[i], GPIO_FUNC_PWM);

        uint slice = pwm_gpio_to_slice_num(pins[i]);
        controller->slice[i] = slice;

        if (!slice_configured[slice]) {
            pwm_set_enabled(slice, false);
            pwm_set_clkdiv(slice, divider);
            pwm_set_wrap(slice, PWM_WRAP);
            slice_configured[slice] = true;
        }

        // Program both channels before enabling their shared slice. Enabling
        // a slice while only its first GPIO is initialized can briefly leave
        // the paired ESC at a stale or zero pulse width.
        pwm_set_gpio_level(pins[i], initial_value);
    }

    for (uint slice = 0; slice < NUM_PWM_SLICES; ++slice) {
        if (slice_configured[slice]) {
            pwm_set_counter(slice, 0);
            pwm_set_enabled(slice, true);
        }
    }
}

void pwm_controller_deinit(struct pwm_controller *controller) {
    bool slice_disabled[NUM_PWM_SLICES] = {false};
    for (uint i = 0; i < controller->num_channels; ++i) {
        uint slice = controller->slice[i];
        if (!slice_disabled[slice]) {
            pwm_set_enabled(slice, false);
            slice_disabled[slice] = true;
        }
        gpio_set_function(controller->pin[i], GPIO_FUNC_SIO);
        gpio_set_dir(controller->pin[i], GPIO_OUT);
        gpio_put(controller->pin[i], 0);
    }
    memset(controller, 0, sizeof(*controller));
}

void pwm_set_throttle(struct pwm_controller *controller, uint channel, uint value) {
    if (channel >= controller->num_channels) {
        return;
    }

    if (value > PWM_WRAP) {
        value = PWM_WRAP;
    }

    pwm_set_gpio_level(controller->pin[channel], value);
}
