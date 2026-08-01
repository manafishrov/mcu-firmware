#ifndef TESTS_SUPPORT_DSHOT_CONTROL_HOST_H
#define TESTS_SUPPORT_DSHOT_CONTROL_HOST_H

#include "dshot_stubs_host.h"
#include "motors.h"
#include <pico/types.h>
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
void dshot_enable_edt_if_idle(const uint16_t *thruster_values, bool *edt_enable_scheduled,
                              absolute_time_t *edt_enable_time,
                              struct dshot_controller *controller0,
                              struct dshot_controller *controller1);
bool dshot_wait_for_telemetry(struct dshot_controller *controller0,
                              struct dshot_controller *controller1);
uint8_t dshot_missing_telemetry_mask(struct dshot_controller *controller0,
                                     struct dshot_controller *controller1);
const char *dshot_dominant_failure_name(const struct dshot_statistics *stats);

#endif
