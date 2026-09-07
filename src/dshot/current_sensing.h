#ifndef CURRENT_SENSING_H
#define CURRENT_SENSING_H

#include "../motors.h"
#include <stdbool.h>
#include <stdint.h>

#define CURRENT_BOARD_COUNT 2
#define CURRENT_NEUTRAL_COMMAND 1000
#define CURRENT_SAMPLE_FRESH_MS 500u
#define CURRENT_IDLE_SETTLE_MS 3000u
#define CURRENT_BASELINE_WINDOW_MS 1000u
#define CURRENT_STABILITY_SPAN_MA 2000
#define CURRENT_UNAVAILABLE_MA (-1)

void current_sensing_reset(void);
void current_sensing_observe_current(uint8_t motor, uint32_t amperes, uint32_t now_ms);
void current_sensing_observe_erpm(uint8_t motor, uint32_t erpm, uint32_t now_ms);
void current_sensing_service(const uint16_t commands[NUM_MOTORS], bool enabled, uint32_t now_ms);
int32_t current_sensing_baseline_ma(uint8_t board);
int32_t current_sensing_current_ma(uint8_t board, uint32_t now_ms);

#endif
