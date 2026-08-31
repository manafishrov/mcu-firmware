#include "support/runtime_config_host.h"
#include "unity/unity.h"
#include "usb_comm.h"

static void test_normalize_dshot_speed_accepts_supported_values(void) {
    TEST_ASSERT_EQUAL_UINT16(150, mcu_runtime_config_normalize_dshot_speed(150));
    TEST_ASSERT_EQUAL_UINT16(300, mcu_runtime_config_normalize_dshot_speed(300));
    TEST_ASSERT_EQUAL_UINT16(600, mcu_runtime_config_normalize_dshot_speed(600));
}

static void test_normalize_dshot_speed_limits_1200_on_non_rp2350_hosts(void) {
    TEST_ASSERT_EQUAL_UINT16(600, mcu_runtime_config_normalize_dshot_speed(1200));
}

static void test_normalize_dshot_speed_defaults_unknown_values_to_300(void) {
    TEST_ASSERT_EQUAL_UINT16(300, mcu_runtime_config_normalize_dshot_speed(450));
    TEST_ASSERT_EQUAL_UINT16(300, mcu_runtime_config_normalize_dshot_speed(0));
}

static void test_validate_keeps_valid_config_unchanged(void) {
    mcu_runtime_config_t config = {
        .protocol = THRUSTER_PROTOCOL_DSHOT,
        .dshot_speed = 600,
    };

    mcu_runtime_config_validate(&config);

    TEST_ASSERT_EQUAL_INT(THRUSTER_PROTOCOL_DSHOT, config.protocol);
    TEST_ASSERT_EQUAL_UINT16(600, config.dshot_speed);
}

static void test_validate_corrects_invalid_protocol_and_speed(void) {
    mcu_runtime_config_t config = {
        .protocol = (thruster_protocol_t)99,
        .dshot_speed = 450,
    };

    mcu_runtime_config_validate(&config);

    TEST_ASSERT_EQUAL_INT(THRUSTER_PROTOCOL_DSHOT, config.protocol);
    TEST_ASSERT_EQUAL_UINT16(300, config.dshot_speed);
}

static void test_detector_reset_is_required_for_dshot_speed_changes(void) {
    const mcu_runtime_config_t current = {
        .protocol = THRUSTER_PROTOCOL_DSHOT,
        .dshot_speed = 300,
    };
    const mcu_runtime_config_t next = {
        .protocol = THRUSTER_PROTOCOL_DSHOT,
        .dshot_speed = 600,
    };

    TEST_ASSERT_TRUE(mcu_runtime_config_requires_detector_reset(&current, &next));
}

static void test_detector_reset_is_required_for_protocol_changes(void) {
    const mcu_runtime_config_t current = {
        .protocol = THRUSTER_PROTOCOL_DSHOT,
        .dshot_speed = 300,
    };
    const mcu_runtime_config_t next = {
        .protocol = THRUSTER_PROTOCOL_PWM,
        .dshot_speed = 300,
    };

    TEST_ASSERT_TRUE(mcu_runtime_config_requires_detector_reset(&current, &next));
}

static void test_detector_reset_ignores_inactive_pwm_dshot_speed(void) {
    const mcu_runtime_config_t current = {
        .protocol = THRUSTER_PROTOCOL_PWM,
        .dshot_speed = 300,
    };
    const mcu_runtime_config_t next = {
        .protocol = THRUSTER_PROTOCOL_PWM,
        .dshot_speed = 600,
    };

    TEST_ASSERT_FALSE(mcu_runtime_config_requires_detector_reset(&current, &next));
}

static void test_parse_packet_accepts_valid_packet_and_populates_config(void) {
    uint8_t packet[USB_CONFIG_PACKET_SIZE] = {
        USB_CONFIG_START_BYTE,
        MCU_CONTROL_COMMAND_APPLY_CONFIG,
        42,
        THRUSTER_PROTOCOL_PWM,
        150,
        0,
        0,
    };
    mcu_control_request_t request = {0};

    packet[USB_CONFIG_PACKET_SIZE - 1] = usb_calculate_checksum(packet, USB_CONFIG_PACKET_SIZE - 1);

    TEST_ASSERT_TRUE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &request));
    TEST_ASSERT_EQUAL_INT(MCU_CONTROL_COMMAND_APPLY_CONFIG, request.command);
    TEST_ASSERT_EQUAL_UINT8(42, request.request_id);
    TEST_ASSERT_EQUAL_INT(THRUSTER_PROTOCOL_PWM, request.config.protocol);
    TEST_ASSERT_EQUAL_UINT16(150, request.config.dshot_speed);
}

static void test_parse_packet_rejects_wrong_size(void) {
    const uint8_t packet[] = {USB_CONFIG_START_BYTE, THRUSTER_PROTOCOL_DSHOT, 0x58, 0x02};
    mcu_control_request_t request = {0};

    TEST_ASSERT_FALSE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &request));
}

static void test_parse_packet_rejects_wrong_start_byte(void) {
    uint8_t packet[USB_CONFIG_PACKET_SIZE] = {
        0x00, MCU_CONTROL_COMMAND_APPLY_CONFIG, 1, THRUSTER_PROTOCOL_DSHOT, 0x58, 0x02, 0,
    };
    mcu_control_request_t request = {0};

    packet[USB_CONFIG_PACKET_SIZE - 1] = usb_calculate_checksum(packet, USB_CONFIG_PACKET_SIZE - 1);

    TEST_ASSERT_FALSE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &request));
}

static void test_build_release_packet_reports_exact_release_identity(void) {
    uint8_t packet[USB_RELEASE_VERSION_MAX_LENGTH + USB_RELEASE_VERSION_PACKET_OVERHEAD] = {0};
    const char expected[] = "1.0.2-rc.3";

    const size_t packet_size = mcu_runtime_config_build_release_packet(packet, sizeof(packet), 42);

    TEST_ASSERT_EQUAL_UINT(sizeof(expected) - 1 + USB_RELEASE_VERSION_PACKET_OVERHEAD, packet_size);
    TEST_ASSERT_EQUAL_HEX8(USB_RELEASE_VERSION_START_BYTE, packet[0]);
    TEST_ASSERT_EQUAL_UINT8(42, packet[1]);
    TEST_ASSERT_EQUAL_UINT8(sizeof(expected) - 1, packet[2]);
    TEST_ASSERT_EQUAL_MEMORY(expected, &packet[3], sizeof(expected) - 1);
    TEST_ASSERT_EQUAL_HEX8(usb_calculate_checksum(packet, packet_size - 1),
                           packet[packet_size - 1]);
}

static void test_build_release_packet_rejects_short_buffer(void) {
    uint8_t packet[4] = {0};

    TEST_ASSERT_EQUAL_UINT(0, mcu_runtime_config_build_release_packet(packet, sizeof(packet), 1));
}

static void test_build_status_packet_reports_runtime_config_without_numeric_version(void) {
    uint8_t packet[USB_RUNTIME_CONFIG_STATUS_PACKET_SIZE] = {0};
    const mcu_runtime_config_t config = {
        .protocol = THRUSTER_PROTOCOL_DSHOT,
        .dshot_speed = 600,
    };
    const uint8_t expected[] = {0xD5,
                                42,
                                MCU_RUNTIME_CONFIG_STATE_APPLIED,
                                MCU_RUNTIME_CONFIG_ERROR_NONE,
                                THRUSTER_PROTOCOL_DSHOT,
                                0x58,
                                0x02,
                                0xA6};

    const size_t packet_size = mcu_runtime_config_build_status_packet(
        packet, sizeof(packet), 42, MCU_RUNTIME_CONFIG_STATE_APPLIED, MCU_RUNTIME_CONFIG_ERROR_NONE,
        &config);

    TEST_ASSERT_EQUAL_UINT(USB_RUNTIME_CONFIG_STATUS_PACKET_SIZE, packet_size);
    TEST_ASSERT_EQUAL_HEX8(USB_RUNTIME_CONFIG_STATUS_START_BYTE, packet[0]);
    TEST_ASSERT_EQUAL_UINT8(THRUSTER_PROTOCOL_DSHOT, packet[4]);
    TEST_ASSERT_EQUAL_UINT16(600, (uint16_t)packet[5] | ((uint16_t)packet[6] << 8));
    TEST_ASSERT_EQUAL_HEX8(usb_calculate_checksum(packet, packet_size - 1),
                           packet[packet_size - 1]);
    TEST_ASSERT_EQUAL_MEMORY(expected, packet, sizeof(expected));
}

static void test_parse_packet_rejects_bad_checksum(void) {
    const uint8_t packet[USB_CONFIG_PACKET_SIZE] = {
        USB_CONFIG_START_BYTE,
        MCU_CONTROL_COMMAND_APPLY_CONFIG,
        1,
        THRUSTER_PROTOCOL_DSHOT,
        0x58,
        0x02,
        0xFF,
    };
    mcu_control_request_t request = {0};

    TEST_ASSERT_FALSE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &request));
}

static void test_protocol_name_returns_expected_strings(void) {
    TEST_ASSERT_EQUAL_STRING("PWM", mcu_runtime_config_protocol_name(THRUSTER_PROTOCOL_PWM));
    TEST_ASSERT_EQUAL_STRING("DShot", mcu_runtime_config_protocol_name(THRUSTER_PROTOCOL_DSHOT));
}

void test_runtime_config(void) {
    RUN_TEST(test_normalize_dshot_speed_accepts_supported_values);
    RUN_TEST(test_normalize_dshot_speed_limits_1200_on_non_rp2350_hosts);
    RUN_TEST(test_normalize_dshot_speed_defaults_unknown_values_to_300);
    RUN_TEST(test_validate_keeps_valid_config_unchanged);
    RUN_TEST(test_validate_corrects_invalid_protocol_and_speed);
    RUN_TEST(test_detector_reset_is_required_for_dshot_speed_changes);
    RUN_TEST(test_detector_reset_is_required_for_protocol_changes);
    RUN_TEST(test_detector_reset_ignores_inactive_pwm_dshot_speed);
    RUN_TEST(test_parse_packet_accepts_valid_packet_and_populates_config);
    RUN_TEST(test_parse_packet_rejects_wrong_size);
    RUN_TEST(test_parse_packet_rejects_wrong_start_byte);
    RUN_TEST(test_build_release_packet_reports_exact_release_identity);
    RUN_TEST(test_build_release_packet_rejects_short_buffer);
    RUN_TEST(test_build_status_packet_reports_runtime_config_without_numeric_version);
    RUN_TEST(test_parse_packet_rejects_bad_checksum);
    RUN_TEST(test_protocol_name_returns_expected_strings);
}
