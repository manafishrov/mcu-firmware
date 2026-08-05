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
        USB_CONFIG_START_BYTE, THRUSTER_PROTOCOL_PWM, 150, 0, 0,
    };
    mcu_runtime_config_t config = {0};

    packet[USB_CONFIG_PACKET_SIZE - 1] = usb_calculate_checksum(packet, USB_CONFIG_PACKET_SIZE - 1);

    TEST_ASSERT_TRUE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &config));
    TEST_ASSERT_EQUAL_INT(THRUSTER_PROTOCOL_PWM, config.protocol);
    TEST_ASSERT_EQUAL_UINT16(150, config.dshot_speed);
}

static void test_parse_packet_rejects_wrong_size(void) {
    const uint8_t packet[] = {USB_CONFIG_START_BYTE, THRUSTER_PROTOCOL_DSHOT, 0x58, 0x02};
    mcu_runtime_config_t config = {0};

    TEST_ASSERT_FALSE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &config));
}

static void test_parse_packet_rejects_wrong_start_byte(void) {
    uint8_t packet[USB_CONFIG_PACKET_SIZE] = {0x00, THRUSTER_PROTOCOL_DSHOT, 0x58, 0x02, 0};
    mcu_runtime_config_t config = {0};

    packet[USB_CONFIG_PACKET_SIZE - 1] = usb_calculate_checksum(packet, USB_CONFIG_PACKET_SIZE - 1);

    TEST_ASSERT_FALSE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &config));
}

static void test_parse_packet_rejects_bad_checksum(void) {
    const uint8_t packet[USB_CONFIG_PACKET_SIZE] = {
        USB_CONFIG_START_BYTE, THRUSTER_PROTOCOL_DSHOT, 0x58, 0x02, 0xFF,
    };
    mcu_runtime_config_t config = {0};

    TEST_ASSERT_FALSE(mcu_runtime_config_parse_packet(packet, sizeof(packet), &config));
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
    RUN_TEST(test_parse_packet_rejects_bad_checksum);
    RUN_TEST(test_protocol_name_returns_expected_strings);
}
