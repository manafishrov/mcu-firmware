#include "dshot/current_sensing.h"
#include "unity/unity.h"

static uint16_t commands[NUM_MOTORS];

static void reset_sensing(void) {
    current_sensing_reset();
    for (int i = 0; i < NUM_MOTORS; ++i) {
        commands[i] = 1000;
    }
}

static void sample(uint32_t now, uint32_t a, uint32_t b, uint32_t erpm) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        current_sensing_observe_current(i, i < 4 ? a : b, now);
        current_sensing_observe_erpm(i, erpm, now);
    }
    current_sensing_service(commands, true, now);
}

static void calibrate(uint32_t baseline) {
    for (uint32_t now = 0; now <= 4200; now += 50) {
        sample(now, baseline, baseline, 0);
    }
}

static void test_variable_baselines_and_decreasing_load(void) {
    const uint32_t baselines[] = {91, 88, 87};
    for (int i = 0; i < 3; ++i) {
        reset_sensing();
        calibrate(baselines[i]);
        TEST_ASSERT_EQUAL_INT32(baselines[i] * 1000, current_sensing_baseline_ma(0));
        TEST_ASSERT_EQUAL_INT32(0, current_sensing_current_ma(0, 4200));
        commands[0] = 1200;
        sample(4250, baselines[i] - 5, baselines[i] - 5, 100);
        TEST_ASSERT_EQUAL_INT32(10000, current_sensing_current_ma(0, 4250) +
                                           current_sensing_current_ma(1, 4250));
        sample(4300, baselines[i] + 1, baselines[i], 100);
        TEST_ASSERT_EQUAL_INT32(0, current_sensing_current_ma(0, 4300));
    }
}

static void test_motion_and_spin_down_cannot_tare(void) {
    reset_sensing();
    commands[0] = 1200;
    for (uint32_t now = 0; now < 5000; now += 50) {
        sample(now, 86, 86, 0);
    }
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(0));
    commands[0] = 1000;
    for (uint32_t now = 5000; now < 10000; now += 50) {
        sample(now, 86, 86, 100);
    }
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(0));
}

static void test_baseline_freezes_and_retare_requires_stable_idle(void) {
    reset_sensing();
    calibrate(91);
    commands[2] = 1300;
    for (uint32_t now = 4250; now < 9000; now += 50) {
        sample(now, 86, 86, 100);
    }
    TEST_ASSERT_EQUAL_INT32(91000, current_sensing_baseline_ma(0));
    commands[2] = 1000;
    for (uint32_t now = 9000; now <= 13200; now += 50) {
        sample(now, 88, 88, 0);
    }
    TEST_ASSERT_EQUAL_INT32(88000, current_sensing_baseline_ma(0));
}

static void test_missing_rpm_or_reused_current_cannot_tare(void) {
    reset_sensing();
    for (uint32_t now = 0; now < 5000; now += 50) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            current_sensing_observe_current(i, 91, now);
        }
        current_sensing_service(commands, true, now);
    }
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(0));
    reset_sensing();
    for (uint32_t now = 0; now <= 5000; now += 10) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            current_sensing_observe_erpm(i, 0, now);
            if (now == 3000) {
                current_sensing_observe_current(i, 91, now);
            }
        }
        current_sensing_service(commands, true, now);
    }
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(0));
}

static void test_unstable_samples_do_not_become_baseline(void) {
    reset_sensing();
    for (uint32_t now = 0; now < 10000; now += 50) {
        sample(now, (now / 50) % 2 ? 80 : 100, 91, 0);
    }
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(0));
    TEST_ASSERT_EQUAL_INT32(91000, current_sensing_baseline_ma(1));
}

static void test_missing_duplicates_expiry_and_reset(void) {
    reset_sensing();
    calibrate(91);
    commands[0] = 1200;
    for (uint32_t now = 4250; now <= 5000; now += 50) {
        current_sensing_observe_current(0, 86, now);
        current_sensing_service(commands, true, now);
    }
    TEST_ASSERT_EQUAL_INT32(5000, current_sensing_current_ma(0, 5000));
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_current_ma(1, 5000));
    // A fresh packet after a telemetry gap must not revive the old baseline.
    current_sensing_observe_current(0, 86, 6000);
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(0));
    current_sensing_service(commands, false, 6000);
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(1));
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_current_ma(0, 6000));
    current_sensing_observe_current(8, 91, 6000);
    current_sensing_observe_current(0, UINT32_MAX, 6000);
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_current_ma(8, 6000));
}

static void test_board_mean_preserves_fraction_and_clamps_each_board(void) {
    reset_sensing();
    for (uint32_t now = 0; now <= 4200; now += 50) {
        sample(now, 91, 87, 0);
    }
    commands[0] = 1100;
    sample(4500, 86, 92, 100);
    current_sensing_observe_current(0, 85, 4501);
    current_sensing_service(commands, true, 4501);
    TEST_ASSERT_EQUAL_INT32(5250, current_sensing_current_ma(0, 4501));
    TEST_ASSERT_EQUAL_INT32(0, current_sensing_current_ma(1, 4501));
}

static void test_motion_between_service_calls_restarts_settle(void) {
    reset_sensing();
    for (uint32_t now = 0; now <= 3900; now += 50) {
        sample(now, 91, 91, 0);
    }
    current_sensing_observe_erpm(0, 100, 3901);
    current_sensing_observe_erpm(0, 0, 3902);
    sample(4000, 91, 91, 0);
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(0));
    TEST_ASSERT_EQUAL_INT32(-1, current_sensing_baseline_ma(1));
}

static void test_time_wrap_and_invalid_samples(void) {
    reset_sensing();
    uint32_t start = UINT32_MAX - 2000;
    for (uint32_t elapsed = 0; elapsed <= 4200; elapsed += 50) {
        sample(start + elapsed, 91, 87, 0);
    }
    TEST_ASSERT_EQUAL_INT32(0, current_sensing_current_ma(0, start + 4200));
    current_sensing_observe_current(NUM_MOTORS, 1, start + 4201);
    current_sensing_observe_current(0, UINT32_MAX, start + 4201);
    current_sensing_observe_current(0, 256, start + 4201);
    TEST_ASSERT_EQUAL_INT32(0, current_sensing_current_ma(0, start + 4201));
}

void test_current_sensing(void) {
    RUN_TEST(test_board_mean_preserves_fraction_and_clamps_each_board);
    RUN_TEST(test_time_wrap_and_invalid_samples);
    RUN_TEST(test_motion_between_service_calls_restarts_settle);
    RUN_TEST(test_variable_baselines_and_decreasing_load);
    RUN_TEST(test_motion_and_spin_down_cannot_tare);
    RUN_TEST(test_baseline_freezes_and_retare_requires_stable_idle);
    RUN_TEST(test_missing_rpm_or_reused_current_cannot_tare);
    RUN_TEST(test_unstable_samples_do_not_become_baseline);
    RUN_TEST(test_missing_duplicates_expiry_and_reset);
}
