/* Include the implementation directly to access static functions */
#include "../src/dshot/dshot.c"
#include "unity/unity.h"

static const uint8_t reverse_gcr_table[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
};

static uint16_t expected_dshot_frame(uint16_t throttle, int telemetry) {
    uint16_t value = (uint16_t)((throttle << 1) | telemetry);
    uint16_t crc = (uint16_t)(~(value ^ (value >> 4) ^ (value >> 8)) & 0x0F);
    return (uint16_t)((value << 4) | crc);
}

static uint32_t encode_gcr20_from_final_word(uint16_t final_word) {
    return ((uint32_t)reverse_gcr_table[(final_word >> 12) & 0x0F] << 15) |
           ((uint32_t)reverse_gcr_table[(final_word >> 8) & 0x0F] << 10) |
           ((uint32_t)reverse_gcr_table[(final_word >> 4) & 0x0F] << 5) |
           reverse_gcr_table[final_word & 0x0F];
}

static uint16_t build_final_word(uint16_t value12) {
    uint16_t crc = (uint16_t)(~(value12 ^ (value12 >> 4) ^ (value12 >> 8)) & 0x0F);
    return (uint16_t)((value12 << 4) | crc);
}

static int gcr20_to_edge_diffs(uint32_t gcr20, uint8_t *edge_diffs) {
    uint32_t stream21 = (1u << 20) | gcr20;
    int edge_count = 0;
    int run_length = 0;

    for (int bit = 20; bit >= 0; --bit) {
        run_length++;
        if (bit == 0 || ((stream21 >> (bit - 1)) & 0x1u) != 0) {
            edge_diffs[edge_count++] = (uint8_t)run_length;
            run_length = 0;
        }
    }

    return edge_count;
}

static void set_simple_run_length_thresholds(void) {
    length_transitions[0] = 0;
    length_transitions[1] = 2;
    length_transitions[2] = 3;
    length_transitions[3] = 4;
}

static void test_dshot_compute_frame_builds_zero_throttle_frame(void) {
    TEST_ASSERT_EQUAL_HEX16(expected_dshot_frame(0, 0), dshot_compute_frame(0, 0));
    TEST_ASSERT_EQUAL_HEX16(0x000F, dshot_compute_frame(0, 0));
}

static void test_dshot_compute_frame_builds_throttle_frame(void) {
    TEST_ASSERT_EQUAL_HEX16(expected_dshot_frame(1000, 0), dshot_compute_frame(1000, 0));
}

static void test_dshot_compute_frame_builds_command_frame(void) {
    TEST_ASSERT_EQUAL_HEX16(expected_dshot_frame(DSHOT_MAX_COMMAND, 1),
                            dshot_compute_frame(DSHOT_MAX_COMMAND, 1));
}

static void test_decode_erpm_returns_zero_for_motor_stopped(void) {
    TEST_ASSERT_EQUAL_UINT32(0, dshot_decode_erpm_telemetry_value(0x0FFF));
}

static void test_decode_erpm_decodes_known_mantissa_value(void) {
    TEST_ASSERT_EQUAL_UINT32(6000, dshot_decode_erpm_telemetry_value(0x0064));
}

static void test_decode_erpm_decodes_period_one_and_six_hundred(void) {
    TEST_ASSERT_EQUAL_UINT32(600000, dshot_decode_erpm_telemetry_value(0x0001));
    TEST_ASSERT_EQUAL_UINT32(1000, dshot_decode_erpm_telemetry_value(0x032C));
}

static void test_decode_erpm_rejects_zero_period(void) {
    TEST_ASSERT_EQUAL_UINT32(DSHOT_TELEMETRY_INVALID, dshot_decode_erpm_telemetry_value(0x0200));
}

static void test_even_nibble_erpm_is_not_misclassified_before_edt_handshake(void) {
    struct dshot_controller controller = {0};
    uint32_t decoded = 0;
    enum dshot_telemetry_type type = DSHOT_TELEMETRY_TYPE_COUNT;

    dshot_decode_telemetry_value(&controller, 0x0201, &decoded, &type);

    TEST_ASSERT_EQUAL_INT(DSHOT_TELEMETRY_TYPE_ERPM, type);
    TEST_ASSERT_EQUAL_UINT32(300000, decoded);
}

static void test_edt_handshake_enables_extended_frame_decoding(void) {
    struct dshot_controller controller = {0};
    struct dshot_motor *motor = &controller.motor[0];
    uint32_t decoded = 0;
    enum dshot_telemetry_type type = DSHOT_TELEMETRY_TYPE_COUNT;

    motor->current_command = DSHOT_EXTENDED_TELEMETRY_ENABLE;
    TEST_ASSERT_TRUE(dshot_handle_edt_handshake(motor, 0x0E00));
    TEST_ASSERT_TRUE(motor->edt_enabled);

    dshot_decode_telemetry_value(&controller, 0x022A, &decoded, &type);

    TEST_ASSERT_EQUAL_INT(DSHOT_TELEMETRY_TYPE_TEMPERATURE, type);
    TEST_ASSERT_EQUAL_UINT32(42, decoded);
}

static void test_edt_current_decodes_in_one_amp_increments(void) {
    struct dshot_controller controller = {0};
    uint32_t decoded = 0;
    enum dshot_telemetry_type type = DSHOT_TELEMETRY_TYPE_COUNT;

    controller.motor[0].edt_enabled = true;
    dshot_decode_telemetry_value(&controller, 0x067B, &decoded, &type);

    TEST_ASSERT_EQUAL_INT(DSHOT_TELEMETRY_TYPE_CURRENT, type);
    TEST_ASSERT_EQUAL_UINT32(123, decoded);
}

static void test_edt_version_response_accepts_nonzero_version(void) {
    struct dshot_motor motor = {.current_command = DSHOT_EXTENDED_TELEMETRY_ENABLE};

    TEST_ASSERT_TRUE(dshot_handle_edt_handshake(&motor, 0x0E21));
    TEST_ASSERT_TRUE(motor.edt_enabled);
}

static void test_edt_marker_is_ignored_without_enable_command(void) {
    struct dshot_motor motor = {0};

    TEST_ASSERT_FALSE(dshot_handle_edt_handshake(&motor, 0x0E00));
    TEST_ASSERT_FALSE(motor.edt_enabled);
}

static void test_non_edt_marker_is_ignored_during_enable_command(void) {
    struct dshot_motor motor = {.current_command = DSHOT_EXTENDED_TELEMETRY_ENABLE};

    TEST_ASSERT_FALSE(dshot_handle_edt_handshake(&motor, 0x0D00));
    TEST_ASSERT_FALSE(motor.edt_enabled);
}

static void test_status_frame_decodes_after_edt_handshake(void) {
    struct dshot_controller controller = {0};
    struct dshot_motor *motor = &controller.motor[0];
    uint32_t decoded = 0;
    enum dshot_telemetry_type type = DSHOT_TELEMETRY_TYPE_COUNT;

    motor->edt_enabled = true;
    dshot_decode_telemetry_value(&controller, 0x0E05, &decoded, &type);

    TEST_ASSERT_EQUAL_INT(DSHOT_TELEMETRY_TYPE_STATE_EVENTS, type);
    TEST_ASSERT_EQUAL_UINT32(5, decoded);
}

static void test_disabling_edt_clears_acknowledgement_and_received_types(void) {
    struct dshot_controller controller = {.num_channels = 1};
    struct dshot_motor *motor = &controller.motor[0];

    motor->edt_enabled = true;
    motor->telemetry_types = UINT8_MAX;

    dshot_command(&controller, 0, DSHOT_EXTENDED_TELEMETRY_DISABLE, 10);

    TEST_ASSERT_FALSE(motor->edt_enabled);
    TEST_ASSERT_EQUAL_UINT8(0, motor->telemetry_types);
}

static void test_decode_gcr_word_accepts_valid_encoded_value(void) {
    uint32_t decoded = 0;
    uint16_t final_word = build_final_word(0x0ABC);
    uint32_t gcr20 = encode_gcr20_from_final_word(final_word);

    TEST_ASSERT_EQUAL_INT(DECODE_OK, decode_gcr_word(gcr20, &decoded));
    TEST_ASSERT_EQUAL_HEX32(final_word, decoded);
}

static void test_decode_gcr_word_rejects_invalid_symbol(void) {
    uint32_t decoded = 0;
    uint16_t final_word = build_final_word(0x0ABC);
    uint32_t gcr20 = encode_gcr20_from_final_word(final_word);

    gcr20 = (gcr20 & 0x07FFFu);

    TEST_ASSERT_EQUAL_INT(DECODE_FAIL_GCR, decode_gcr_word(gcr20, &decoded));
}

static void test_decode_gcr_word_rejects_bad_crc(void) {
    uint32_t decoded = 0;
    uint16_t good_word = build_final_word(0x0ABC);
    uint16_t final_word = (uint16_t)((good_word & 0xFFF0u) | (((good_word & 0x0Fu) + 1u) & 0x0Fu));
    uint32_t gcr20 = encode_gcr20_from_final_word(final_word);

    TEST_ASSERT_EQUAL_INT(DECODE_FAIL_CRC, decode_gcr_word(gcr20, &decoded));
}

static void test_dshot_get_telemetry_quality_percent_handles_missing_erpm(void) {
    struct dshot_controller controller = {0};

    controller.num_channels = 1;
    controller.motor[0].quality.packet_count_sum = 10;

    TEST_ASSERT_EQUAL_INT16(0, dshot_get_telemetry_quality_percent(&controller, 0));
}

static void test_dshot_get_telemetry_quality_percent_reports_full_quality(void) {
    struct dshot_controller controller = {0};

    controller.num_channels = 1;
    controller.motor[0].telemetry_types = (1u << DSHOT_TELEMETRY_TYPE_ERPM);
    controller.motor[0].quality.packet_count_sum = 8;
    controller.motor[0].quality.invalid_count_sum = 0;

    TEST_ASSERT_EQUAL_INT16(10000, dshot_get_telemetry_quality_percent(&controller, 0));
}

static void test_dshot_get_telemetry_quality_percent_reports_half_invalid_packets(void) {
    struct dshot_controller controller = {0};

    controller.num_channels = 1;
    controller.motor[0].telemetry_types = (1u << DSHOT_TELEMETRY_TYPE_ERPM);
    controller.motor[0].quality.packet_count_sum = 10;
    controller.motor[0].quality.invalid_count_sum = 5;

    TEST_ASSERT_EQUAL_INT16(5000, dshot_get_telemetry_quality_percent(&controller, 0));
}

static void test_dshot_get_telemetry_quality_percent_rejects_invalid_channel(void) {
    struct dshot_controller controller = {0};

    controller.num_channels = 1;

    TEST_ASSERT_EQUAL_INT16(0, dshot_get_telemetry_quality_percent(&controller, 1));
}

static void test_build_gcr_word_rejects_invalid_edge_counts(void) {
    uint32_t gcr20 = 0;
    uint8_t edge_diffs[22] = {0};

    TEST_ASSERT_EQUAL_INT(DECODE_FAIL_EDGE_COUNT, build_gcr_word(edge_diffs, 1, &gcr20));
    TEST_ASSERT_EQUAL_INT(DECODE_FAIL_EDGE_COUNT, build_gcr_word(edge_diffs, 22, &gcr20));
}

static void test_build_gcr_word_builds_expected_word_from_valid_edges(void) {
    uint16_t final_word = build_final_word(0x0ABC);
    uint32_t target_gcr20 = encode_gcr20_from_final_word(final_word);
    uint8_t edge_diffs[MAX_EDGES] = {0};
    uint32_t built_word = 0;
    int edge_count;

    set_simple_run_length_thresholds();
    edge_count = gcr20_to_edge_diffs(target_gcr20, edge_diffs);

    TEST_ASSERT_EQUAL_INT(DECODE_OK, build_gcr_word(edge_diffs, edge_count, &built_word));
    TEST_ASSERT_EQUAL_HEX32(1u << 20, built_word & (1u << 20));
    TEST_ASSERT_EQUAL_HEX32(target_gcr20, built_word & 0xFFFFFu);
}

void test_dshot_protocol(void) {
    RUN_TEST(test_dshot_compute_frame_builds_zero_throttle_frame);
    RUN_TEST(test_dshot_compute_frame_builds_throttle_frame);
    RUN_TEST(test_dshot_compute_frame_builds_command_frame);
    RUN_TEST(test_decode_erpm_returns_zero_for_motor_stopped);
    RUN_TEST(test_decode_erpm_decodes_known_mantissa_value);
    RUN_TEST(test_decode_erpm_decodes_period_one_and_six_hundred);
    RUN_TEST(test_decode_erpm_rejects_zero_period);
    RUN_TEST(test_even_nibble_erpm_is_not_misclassified_before_edt_handshake);
    RUN_TEST(test_edt_handshake_enables_extended_frame_decoding);
    RUN_TEST(test_edt_current_decodes_in_one_amp_increments);
    RUN_TEST(test_edt_version_response_accepts_nonzero_version);
    RUN_TEST(test_edt_marker_is_ignored_without_enable_command);
    RUN_TEST(test_non_edt_marker_is_ignored_during_enable_command);
    RUN_TEST(test_status_frame_decodes_after_edt_handshake);
    RUN_TEST(test_disabling_edt_clears_acknowledgement_and_received_types);
    RUN_TEST(test_decode_gcr_word_accepts_valid_encoded_value);
    RUN_TEST(test_decode_gcr_word_rejects_invalid_symbol);
    RUN_TEST(test_decode_gcr_word_rejects_bad_crc);
    RUN_TEST(test_dshot_get_telemetry_quality_percent_handles_missing_erpm);
    RUN_TEST(test_dshot_get_telemetry_quality_percent_reports_full_quality);
    RUN_TEST(test_dshot_get_telemetry_quality_percent_reports_half_invalid_packets);
    RUN_TEST(test_dshot_get_telemetry_quality_percent_rejects_invalid_channel);
    RUN_TEST(test_build_gcr_word_rejects_invalid_edge_counts);
    RUN_TEST(test_build_gcr_word_builds_expected_word_from_valid_edges);
}
