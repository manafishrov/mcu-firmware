#include "current_sensing.h"
#include "../motors.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CHANNELS_PER_BOARD 4
#define CURRENT_MIN_WINDOW_SAMPLES 10u
#define CURRENT_MAX_WINDOW_SAMPLES 1000u

_Static_assert(NUM_MOTORS == CURRENT_BOARD_COUNT * CHANNELS_PER_BOARD,
               "Current sensing requires two four-in-one ESC boards");
_Static_assert(NUM_MOTORS_0 == CHANNELS_PER_BOARD, "First board must have four controllers");
_Static_assert(NUM_MOTORS_1 == CHANNELS_PER_BOARD, "Second board must have four controllers");

typedef struct {
    uint32_t value;
    uint32_t time_ms;
    bool valid;
} sample_t;

typedef struct {
    int32_t baseline_ma;
    uint32_t window_start_ms;
    int32_t minimum_ma;
    int32_t maximum_ma;
    int64_t sum_ma;
    uint16_t samples;
    uint8_t updated_mask;
    bool collecting;
} board_t;

static sample_t raw[NUM_MOTORS];
static sample_t rpm[NUM_MOTORS];
static board_t boards[CURRENT_BOARD_COUNT];
static bool idle;
static uint32_t idle_start_ms;

static bool fresh(const sample_t *sample, uint32_t now_ms) {
    return sample->valid && now_ms - sample->time_ms <= CURRENT_SAMPLE_FRESH_MS;
}

static void reset_window(board_t *board) {
    board->collecting = false;
    board->samples = 0;
    board->sum_ma = 0;
    board->minimum_ma = INT32_MAX;
    board->maximum_ma = 0;
    board->updated_mask = 0;
}

void current_sensing_reset(void) {
    memset(raw, 0, sizeof(raw));
    memset(rpm, 0, sizeof(rpm));
    memset(boards, 0, sizeof(boards));
    idle = false;
    for (int b = 0; b < CURRENT_BOARD_COUNT; ++b) {
        boards[b].baseline_ma = CURRENT_UNAVAILABLE_MA;
        reset_window(&boards[b]);
    }
}

void current_sensing_observe_current(uint8_t motor, uint32_t amperes, uint32_t now_ms) {
    if (motor >= NUM_MOTORS || amperes > UINT8_MAX) {
        return;
    }
    board_t *board = &boards[motor / CHANNELS_PER_BOARD];
    if (board->baseline_ma >= 0) {
        bool previous_fresh = false;
        int first = (motor / CHANNELS_PER_BOARD) * CHANNELS_PER_BOARD;
        for (int i = first; i < first + CHANNELS_PER_BOARD; ++i) {
            previous_fresh = previous_fresh || fresh(&raw[i], now_ms);
        }
        if (!previous_fresh) {
            board->baseline_ma = CURRENT_UNAVAILABLE_MA;
            reset_window(board);
        }
    }
    raw[motor] = (sample_t){.value = amperes, .time_ms = now_ms, .valid = true};
    boards[motor / CHANNELS_PER_BOARD].updated_mask |=
        (uint8_t)(1u << (motor % CHANNELS_PER_BOARD));
}

void current_sensing_observe_erpm(uint8_t motor, uint32_t erpm, uint32_t now_ms) {
    if (motor < NUM_MOTORS) {
        bool continuous = fresh(&rpm[motor], now_ms);
        rpm[motor] = (sample_t){.value = erpm, .time_ms = now_ms, .valid = true};
        if (erpm != 0 || !continuous) {
            idle = false;
            for (int board = 0; board < CURRENT_BOARD_COUNT; ++board) {
                reset_window(&boards[board]);
            }
        }
    }
}

static int32_t board_mean(uint8_t board, uint32_t now_ms, uint8_t *count) {
    int32_t sum_ma = 0;
    *count = 0;
    for (int i = board * CHANNELS_PER_BOARD; i < (board + 1) * CHANNELS_PER_BOARD; ++i) {
        if (fresh(&raw[i], now_ms)) {
            sum_ma += (int32_t)raw[i].value * 1000;
            ++*count;
        }
    }
    return *count ? sum_ma / *count : CURRENT_UNAVAILABLE_MA;
}

static bool motors_stopped(const uint16_t commands[NUM_MOTORS], uint32_t now_ms) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (commands[i] != CURRENT_NEUTRAL_COMMAND || !fresh(&rpm[i], now_ms) ||
            rpm[i].value != 0) {
            return false;
        }
    }
    return true;
}

void current_sensing_service(const uint16_t commands[NUM_MOTORS], bool enabled, uint32_t now_ms) {
    if (!enabled) {
        current_sensing_reset();
        return;
    }
    bool stopped = motors_stopped(commands, now_ms);
    if (!stopped) {
        idle = false;
    } else if (!idle) {
        idle = true;
        idle_start_ms = now_ms;
    }
    for (uint8_t b = 0; b < CURRENT_BOARD_COUNT; ++b) {
        board_t *board = &boards[b];
        uint8_t count;
        int32_t mean = board_mean(b, now_ms, &count);
        if (!count) {
            board->baseline_ma = CURRENT_UNAVAILABLE_MA;
        }
        if (!idle || now_ms - idle_start_ms < CURRENT_IDLE_SETTLE_MS ||
            count != CHANNELS_PER_BOARD) {
            reset_window(board);
            continue;
        }
        if (!board->collecting) {
            reset_window(board);
            board->collecting = true;
            board->window_start_ms = now_ms;
            continue;
        }
        // A sample requires new observations from every controller. A fast
        // service loop must not turn a single old packet into a stable window.
        if (board->updated_mask != 0x0Fu) {
            continue;
        }
        board->updated_mask = 0;
        if (mean < board->minimum_ma) {
            board->minimum_ma = mean;
        }
        if (mean > board->maximum_ma) {
            board->maximum_ma = mean;
        }
        if (board->maximum_ma - board->minimum_ma > CURRENT_STABILITY_SPAN_MA ||
            board->samples >= CURRENT_MAX_WINDOW_SAMPLES) {
            reset_window(board);
            continue;
        }
        board->sum_ma += mean;
        ++board->samples;
        if (now_ms - board->window_start_ms >= CURRENT_BASELINE_WINDOW_MS &&
            board->samples >= CURRENT_MIN_WINDOW_SAMPLES) {
            board->baseline_ma = (int32_t)(board->sum_ma / board->samples);
            reset_window(board);
        }
    }
}

int32_t current_sensing_baseline_ma(uint8_t board) {
    return board < CURRENT_BOARD_COUNT ? boards[board].baseline_ma : CURRENT_UNAVAILABLE_MA;
}

int32_t current_sensing_current_ma(uint8_t board, uint32_t now_ms) {
    if (board >= CURRENT_BOARD_COUNT || boards[board].baseline_ma < 0) {
        return CURRENT_UNAVAILABLE_MA;
    }
    uint8_t count;
    int32_t mean = board_mean(board, now_ms, &count);
    if (!count) {
        return CURRENT_UNAVAILABLE_MA;
    }
    int32_t corrected = boards[board].baseline_ma - mean;
    return corrected > 0 ? corrected : 0;
}
