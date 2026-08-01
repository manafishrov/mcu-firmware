#include "support/dshot_control_host.h"
#include "unity/unity.h"

static void init_dshot_controllers(struct dshot_controller *controller0,
                                   struct dshot_controller *controller1) {
    *controller0 = (struct dshot_controller){.num_channels = NUM_MOTORS_0};
    *controller1 = (struct dshot_controller){.num_channels = NUM_MOTORS_1};
}

static void test_translate_throttle_to_command_maps_neutral_to_zero(void) {
    TEST_ASSERT_EQUAL_UINT16(DSHOT_CMD_NEUTRAL,
                             dshot_translate_throttle_to_command(CMD_THROTTLE_NEUTRAL));
}

static void test_translate_throttle_to_command_maps_forward_range(void) {
    TEST_ASSERT_EQUAL_UINT16(DSHOT_CMD_MIN_FORWARD,
                             dshot_translate_throttle_to_command(CMD_THROTTLE_NEUTRAL + 1));
    TEST_ASSERT_EQUAL_UINT16(DSHOT_CMD_MAX_FORWARD,
                             dshot_translate_throttle_to_command(CMD_THROTTLE_MAX_FORWARD));
}

static void test_translate_throttle_to_command_maps_reverse_range(void) {
    TEST_ASSERT_EQUAL_UINT16(DSHOT_CMD_MIN_REVERSE,
                             dshot_translate_throttle_to_command(CMD_THROTTLE_NEUTRAL - 1));
    TEST_ASSERT_EQUAL_UINT16(DSHOT_CMD_MAX_REVERSE,
                             dshot_translate_throttle_to_command(CMD_THROTTLE_MIN_REVERSE));
}

static void test_translate_throttle_to_command_rejects_out_of_range_values(void) {
    TEST_ASSERT_EQUAL_UINT16(DSHOT_CMD_NEUTRAL, dshot_translate_throttle_to_command(2001));
}

static void test_edt_enable_is_scheduled_while_acknowledgement_is_missing(void) {
    const uint16_t thruster_values[NUM_MOTORS] = {
        CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL,
        CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL,
    };
    bool scheduled[NUM_MOTORS] = {true, true, true, true, true, true, true, true};
    absolute_time_t enable_time[NUM_MOTORS] = {0};
    struct dshot_controller controller0;
    struct dshot_controller controller1;

    init_dshot_controllers(&controller0, &controller1);
    dshot_enable_edt_if_idle(thruster_values, scheduled, enable_time, &controller0, &controller1);

    for (int i = 0; i < NUM_MOTORS_0; ++i) {
        TEST_ASSERT_EQUAL_UINT16(DSHOT_EXTENDED_TELEMETRY_ENABLE,
                                 controller0.motor[i].current_command);
        TEST_ASSERT_EQUAL_UINT8(10, controller0.motor[i].command_counter);
    }
    for (int i = 0; i < NUM_MOTORS_1; ++i) {
        TEST_ASSERT_EQUAL_UINT16(DSHOT_EXTENDED_TELEMETRY_ENABLE,
                                 controller1.motor[i].current_command);
        TEST_ASSERT_EQUAL_UINT8(10, controller1.motor[i].command_counter);
    }
}

static void test_edt_acknowledgement_stops_enable_retries(void) {
    const uint16_t thruster_values[NUM_MOTORS] = {
        CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL,
        CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL,
    };
    bool scheduled[NUM_MOTORS] = {true, true, true, true, true, true, true, true};
    absolute_time_t enable_time[NUM_MOTORS] = {0};
    struct dshot_controller controller0;
    struct dshot_controller controller1;

    init_dshot_controllers(&controller0, &controller1);
    for (int i = 0; i < NUM_MOTORS_0; ++i) {
        controller0.motor[i].edt_enabled = true;
    }
    for (int i = 0; i < NUM_MOTORS_1; ++i) {
        controller1.motor[i].edt_enabled = true;
    }

    dshot_enable_edt_if_idle(thruster_values, scheduled, enable_time, &controller0, &controller1);

    for (int i = 0; i < NUM_MOTORS; ++i) {
        TEST_ASSERT_FALSE(scheduled[i]);
    }
    for (int i = 0; i < NUM_MOTORS_0; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0, controller0.motor[i].command_counter);
    }
    for (int i = 0; i < NUM_MOTORS_1; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0, controller1.motor[i].command_counter);
    }
}

static void test_edt_enable_waits_until_all_thrusters_are_idle(void) {
    uint16_t thruster_values[NUM_MOTORS] = {
        CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL,
        CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL, CMD_THROTTLE_NEUTRAL,
    };
    bool scheduled[NUM_MOTORS] = {true, true, true, true, true, true, true, true};
    absolute_time_t enable_time[NUM_MOTORS] = {0};
    struct dshot_controller controller0;
    struct dshot_controller controller1;

    init_dshot_controllers(&controller0, &controller1);
    thruster_values[3] = CMD_THROTTLE_NEUTRAL + 1;

    dshot_enable_edt_if_idle(thruster_values, scheduled, enable_time, &controller0, &controller1);

    for (int i = 0; i < NUM_MOTORS; ++i) {
        TEST_ASSERT_FALSE(scheduled[i]);
    }
    for (int i = 0; i < NUM_MOTORS_0; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0, controller0.motor[i].command_counter);
    }
    for (int i = 0; i < NUM_MOTORS_1; ++i) {
        TEST_ASSERT_EQUAL_UINT8(0, controller1.motor[i].command_counter);
    }
}

static void test_wait_for_telemetry_succeeds_when_every_motor_has_erpm(void) {
    struct dshot_controller controller0;
    struct dshot_controller controller1;
    init_dshot_controllers(&controller0, &controller1);

    for (int i = 0; i < NUM_MOTORS_0; ++i) {
        controller0.motor[i].telemetry_types = 1 << DSHOT_TELEMETRY_TYPE_ERPM;
    }
    for (int i = 0; i < NUM_MOTORS_1; ++i) {
        controller1.motor[i].telemetry_types = 1 << DSHOT_TELEMETRY_TYPE_ERPM;
    }

    TEST_ASSERT_TRUE(dshot_wait_for_telemetry(&controller0, &controller1));
    TEST_ASSERT_EQUAL_UINT8(0, dshot_missing_telemetry_mask(&controller0, &controller1));
}

static void test_missing_telemetry_mask_identifies_each_motor(void) {
    struct dshot_controller controller0;
    struct dshot_controller controller1;
    init_dshot_controllers(&controller0, &controller1);

    for (int i = 0; i < NUM_MOTORS_0; ++i) {
        controller0.motor[i].telemetry_types = 1 << DSHOT_TELEMETRY_TYPE_ERPM;
    }
    for (int i = 0; i < NUM_MOTORS_1; ++i) {
        controller1.motor[i].telemetry_types = 1 << DSHOT_TELEMETRY_TYPE_ERPM;
    }
    controller0.motor[2].telemetry_types = 0;
    controller1.motor[1].telemetry_types = 0;

    TEST_ASSERT_EQUAL_HEX8((1u << 2) | (1u << (NUM_MOTORS_0 + 1)),
                           dshot_missing_telemetry_mask(&controller0, &controller1));
}

static void test_dominant_failure_name_defaults_to_timeout(void) {
    const struct dshot_statistics stats = {0};

    TEST_ASSERT_EQUAL_STRING("timeout", dshot_dominant_failure_name(&stats));
}

static void test_dominant_failure_name_reports_bad_gcr_when_highest(void) {
    const struct dshot_statistics stats = {.rx_timeout = 1, .rx_bad_gcr = 2};

    TEST_ASSERT_EQUAL_STRING("bad_gcr", dshot_dominant_failure_name(&stats));
}

static void test_dominant_failure_name_reports_bad_crc_when_highest(void) {
    const struct dshot_statistics stats = {.rx_timeout = 1, .rx_bad_gcr = 2, .rx_bad_crc = 3};

    TEST_ASSERT_EQUAL_STRING("bad_crc", dshot_dominant_failure_name(&stats));
}

static void test_dominant_failure_name_reports_bad_type_when_highest(void) {
    const struct dshot_statistics stats = {
        .rx_timeout = 1,
        .rx_bad_gcr = 2,
        .rx_bad_crc = 3,
        .rx_bad_type = 4,
    };

    TEST_ASSERT_EQUAL_STRING("bad_type", dshot_dominant_failure_name(&stats));
}

static void test_dominant_failure_name_keeps_timeout_when_it_is_highest(void) {
    const struct dshot_statistics stats = {
        .rx_timeout = 5,
        .rx_bad_gcr = 4,
        .rx_bad_crc = 3,
        .rx_bad_type = 2,
    };

    TEST_ASSERT_EQUAL_STRING("timeout", dshot_dominant_failure_name(&stats));
}

void test_dshot_control(void) {
    RUN_TEST(test_translate_throttle_to_command_maps_neutral_to_zero);
    RUN_TEST(test_translate_throttle_to_command_maps_forward_range);
    RUN_TEST(test_translate_throttle_to_command_maps_reverse_range);
    RUN_TEST(test_translate_throttle_to_command_rejects_out_of_range_values);
    RUN_TEST(test_edt_enable_is_scheduled_while_acknowledgement_is_missing);
    RUN_TEST(test_edt_acknowledgement_stops_enable_retries);
    RUN_TEST(test_edt_enable_waits_until_all_thrusters_are_idle);
    RUN_TEST(test_wait_for_telemetry_succeeds_when_every_motor_has_erpm);
    RUN_TEST(test_missing_telemetry_mask_identifies_each_motor);
    RUN_TEST(test_dominant_failure_name_defaults_to_timeout);
    RUN_TEST(test_dominant_failure_name_reports_bad_gcr_when_highest);
    RUN_TEST(test_dominant_failure_name_reports_bad_crc_when_highest);
    RUN_TEST(test_dominant_failure_name_reports_bad_type_when_highest);
    RUN_TEST(test_dominant_failure_name_keeps_timeout_when_it_is_highest);
}
