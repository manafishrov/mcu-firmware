#ifndef DSHOT_CONTROL_H
#define DSHOT_CONTROL_H

#include "../motors.h"
#include "dshot.h"
#include <pico/time.h>
#include <stdbool.h>
#include <stdint.h>

#define CMD_THROTTLE_MIN_REVERSE 0
#define CMD_THROTTLE_NEUTRAL 1000
#define CMD_THROTTLE_MAX_FORWARD 2000
#define DSHOT_CMD_NEUTRAL 0
#define DSHOT_CMD_MIN_REVERSE 48
#define DSHOT_CMD_MAX_REVERSE 1047
#define DSHOT_CMD_MIN_FORWARD 1048
#define DSHOT_CMD_MAX_FORWARD 2047

uint16_t dshot_translate_throttle_to_command(uint16_t cmd_throttle);
void dshot_get_motor_controller(int motor_index, struct dshot_controller **ctrl, int *channel,
                                struct dshot_controller *controller0,
                                struct dshot_controller *controller1);
void dshot_run_until_idle(struct dshot_controller *controller0,
                          struct dshot_controller *controller1);
void dshot_run_frame_cycles(struct dshot_controller *controller0,
                            struct dshot_controller *controller1, int cycles);
void dshot_send_command_to_all(struct dshot_controller *controller0,
                               struct dshot_controller *controller1, uint16_t command,
                               uint8_t repeat_count);
void dshot_enable_edt_if_idle(const uint16_t *thruster_values, bool *edt_enable_scheduled,
                              absolute_time_t *edt_enable_time,
                              struct dshot_controller *controller0,
                              struct dshot_controller *controller1);
void dshot_send_commands(uint16_t *thruster_values, struct dshot_controller *controller0,
                         struct dshot_controller *controller1);
bool dshot_wait_for_telemetry(struct dshot_controller *controller0,
                              struct dshot_controller *controller1);
uint8_t dshot_missing_telemetry_mask(struct dshot_controller *controller0,
                                     struct dshot_controller *controller1);
bool dshot_quality_report_due(absolute_time_t *next_quality_report_time,
                              uint32_t quality_report_interval_ms, absolute_time_t now);
const char *dshot_dominant_failure_name(const struct dshot_statistics *stats);

#endif
