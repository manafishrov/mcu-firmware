/*
 * Standard PWM motor controller for RP2040.
 * Generates 50Hz servo-style PWM signals for ESC control.
 * Alternative to DShot for ESCs that only support analog PWM input.
 */

#ifndef PWM_H
#define PWM_H

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <stdint.h>

#define PWM_MAX_CHANNELS 8
#define PWM_FREQUENCY 50    /* Standard servo/ESC PWM frequency (Hz) */
#define PWM_PERIOD_US 20000 /* 1 / 50Hz = 20ms */
#define PWM_STEPS PWM_PERIOD_US
#define PWM_WRAP (PWM_STEPS - 1)

struct pwm_controller {
    uint slice[PWM_MAX_CHANNELS];
    uint pin[PWM_MAX_CHANNELS];
    uint num_channels;
};

void pwm_controller_init(struct pwm_controller *controller, uint *pins, uint num_channels,
                         uint initial_value);
void pwm_controller_deinit(struct pwm_controller *controller);

/* Set pulse width in microseconds (typically 1000-2000us for ESCs) */
void pwm_set_throttle(struct pwm_controller *controller, uint channel, uint value);

#endif
