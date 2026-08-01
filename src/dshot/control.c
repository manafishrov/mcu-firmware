#include "control.h"
#include "../motors.h"
#include "dshot.h"
#include "telemetry_usb.h"
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stdint.h>

uint16_t dshot_translate_throttle_to_command(uint16_t cmd_throttle) {
    if (cmd_throttle == CMD_THROTTLE_NEUTRAL) {
        return DSHOT_CMD_NEUTRAL;
    }
    if (cmd_throttle > CMD_THROTTLE_NEUTRAL && cmd_throttle <= CMD_THROTTLE_MAX_FORWARD) {
        return (cmd_throttle - CMD_THROTTLE_NEUTRAL - 1) + DSHOT_CMD_MIN_FORWARD;
    }
    if (cmd_throttle < CMD_THROTTLE_NEUTRAL && cmd_throttle >= CMD_THROTTLE_MIN_REVERSE) {
        return DSHOT_CMD_MAX_REVERSE - cmd_throttle;
    }
    return DSHOT_CMD_NEUTRAL;
}

static bool controller_has_pending_work(const struct dshot_controller *controller) {
    for (int i = 0; i < controller->num_channels; ++i) {
        if (controller->motor[i].command_counter > 0) {
            return true;
        }
    }
    return false;
}

void dshot_get_motor_controller(int motor_index, struct dshot_controller **ctrl, int *channel,
                                struct dshot_controller *controller0,
                                struct dshot_controller *controller1) {
    *ctrl = (motor_index < NUM_MOTORS_0) ? controller0 : controller1;
    *channel = (motor_index < NUM_MOTORS_0) ? motor_index : (motor_index - NUM_MOTORS_0);
}

void dshot_run_until_idle(struct dshot_controller *controller0,
                          struct dshot_controller *controller1) {
    while (controller_has_pending_work(controller0) || controller_has_pending_work(controller1)) {
        dshot_loop(controller0);
        dshot_loop(controller1);
    }
}

void dshot_run_frame_cycles(struct dshot_controller *controller0,
                            struct dshot_controller *controller1, int cycles) {
    for (int i = 0; i < cycles; ++i) {
        dshot_loop(controller0);
        dshot_loop(controller1);
    }
}

void dshot_send_command_to_all(struct dshot_controller *controller0,
                               struct dshot_controller *controller1, uint16_t command,
                               uint8_t repeat_count) {
    sleep_us(10000);
    for (int repeat = 0; repeat < repeat_count; ++repeat) {
        for (int i = 0; i < NUM_MOTORS; ++i) {
            struct dshot_controller *ctrl;
            int channel;
            dshot_get_motor_controller(i, &ctrl, &channel, controller0, controller1);
            dshot_command(ctrl, channel, command, 1);
        }

        dshot_run_until_idle(controller0, controller1);
        dshot_telemetry_usb_flush();
        if (repeat + 1 < repeat_count) {
            sleep_us(1000);
        }
    }

    if (command == DSHOT_CMD_SAVE_SETTINGS) {
        sleep_ms(35);
    } else if (command == DSHOT_CMD_ESC_INFO) {
        sleep_ms(12);
    }
}

void dshot_enable_edt_if_idle(const uint16_t *thruster_values, bool *edt_enable_scheduled,
                              absolute_time_t *edt_enable_time,
                              struct dshot_controller *controller0,
                              struct dshot_controller *controller1) {
    absolute_time_t now = get_absolute_time();

    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (thruster_values[i] != CMD_THROTTLE_NEUTRAL) {
            for (int j = 0; j < NUM_MOTORS; ++j) {
                edt_enable_scheduled[j] = false;
            }
            return;
        }
    }

    for (int i = 0; i < NUM_MOTORS; ++i) {
        struct dshot_controller *ctrl;
        int channel;
        dshot_get_motor_controller(i, &ctrl, &channel, controller0, controller1);
        struct dshot_motor *motor = &ctrl->motor[channel];

        if (motor->edt_enabled) {
            edt_enable_scheduled[i] = false;
            continue;
        }

        if (!edt_enable_scheduled[i]) {
            edt_enable_scheduled[i] = true;
            edt_enable_time[i] = delayed_by_us(now, 10000);
        }

        if (motor->current_command == 0 && absolute_time_diff_us(edt_enable_time[i], now) >= 0) {
            dshot_command(ctrl, channel, DSHOT_EXTENDED_TELEMETRY_ENABLE, 10);
            edt_enable_time[i] = delayed_by_us(now, 10000);
        }
    }
}

void dshot_send_commands(uint16_t *thruster_values, struct dshot_controller *controller0,
                         struct dshot_controller *controller1) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct dshot_controller *ctrl;
        int channel;
        dshot_get_motor_controller(i, &ctrl, &channel, controller0, controller1);
        dshot_throttle(ctrl, channel, dshot_translate_throttle_to_command(thruster_values[i]));
    }
}

bool dshot_wait_for_telemetry(struct dshot_controller *controller0,
                              struct dshot_controller *controller1) {
    absolute_time_t deadline = delayed_by_ms(get_absolute_time(), 500);
    while (dshot_missing_telemetry_mask(controller0, controller1) != 0) {
        dshot_loop(controller0);
        dshot_loop(controller1);
        dshot_telemetry_usb_flush();
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
    }
    return true;
}

uint8_t dshot_missing_telemetry_mask(struct dshot_controller *controller0,
                                     struct dshot_controller *controller1) {
    uint8_t missing = 0;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        struct dshot_controller *ctrl;
        int channel;
        dshot_get_motor_controller(i, &ctrl, &channel, controller0, controller1);
        if ((ctrl->motor[channel].telemetry_types & (1u << DSHOT_TELEMETRY_TYPE_ERPM)) == 0) {
            missing |= (uint8_t)(1u << i);
        }
    }
    return missing;
}

bool dshot_quality_report_due(absolute_time_t *next_quality_report_time,
                              uint32_t quality_report_interval_ms, absolute_time_t now) {
    if (absolute_time_diff_us(*next_quality_report_time, now) < 0) {
        return false;
    }

    do {
        *next_quality_report_time =
            delayed_by_ms(*next_quality_report_time, quality_report_interval_ms);
    } while (absolute_time_diff_us(*next_quality_report_time, now) >= 0);

    return true;
}

const char *dshot_dominant_failure_name(const struct dshot_statistics *stats) {
    uint32_t max_val = stats->rx_timeout;
    const char *name = "timeout";
    if (stats->rx_bad_gcr > max_val) {
        max_val = stats->rx_bad_gcr;
        name = "bad_gcr";
    }
    if (stats->rx_bad_crc > max_val) {
        max_val = stats->rx_bad_crc;
        name = "bad_crc";
    }
    if (stats->rx_bad_type > max_val) {
        name = "bad_type";
    }
    return name;
}
