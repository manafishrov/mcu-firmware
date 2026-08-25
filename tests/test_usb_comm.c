#include "unity/unity.h"
#include "usb_comm.h"

static void test_usb_calculate_checksum_returns_zero_for_empty_data(void) {
    TEST_ASSERT_EQUAL_HEX8(0x00, usb_calculate_checksum(0, 0));
}

static void test_usb_calculate_checksum_returns_single_byte_value(void) {
    const uint8_t data[] = {0xA5};

    TEST_ASSERT_EQUAL_HEX8(0xA5, usb_calculate_checksum(data, sizeof(data)));
}

static void test_usb_calculate_checksum_applies_xor_across_known_sequence(void) {
    const uint8_t data[] = {0x5A, 0xE8, 0x03, 0xD0};

    TEST_ASSERT_EQUAL_HEX8(0x61, usb_calculate_checksum(data, sizeof(data)));
}

static void test_usb_calculate_checksum_matches_known_multi_byte_result(void) {
    const uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};

    TEST_ASSERT_EQUAL_HEX8(0x01, usb_calculate_checksum(data, sizeof(data)));
}

static void test_usb_parse_packet_accepts_valid_packet_and_extracts_values(void) {
    uint8_t packet[USB_INPUT_PACKET_SIZE(3)] = {
        USB_INPUT_START_BYTE, 0x34, 0x12, 0xCD, 0xAB, 0x01, 0x00, 0x00,
    };
    uint16_t raw_values[3] = {0};
    absolute_time_t last_comm_time = 42;

    packet[sizeof(packet) - 1] = usb_calculate_checksum(packet, sizeof(packet) - 1);

    TEST_ASSERT_TRUE(usb_parse_packet(packet, sizeof(packet), raw_values, 3, &last_comm_time));
    TEST_ASSERT_EQUAL_UINT16(0x1234, raw_values[0]);
    TEST_ASSERT_EQUAL_UINT16(0xABCD, raw_values[1]);
    TEST_ASSERT_EQUAL_UINT16(0x0001, raw_values[2]);
    TEST_ASSERT_EQUAL_UINT64(0, last_comm_time);
}

static void test_usb_parse_packet_rejects_wrong_start_byte(void) {
    uint8_t packet[USB_INPUT_PACKET_SIZE(2)] = {
        0x00, 0xE8, 0x03, 0xD0, 0x07, 0x00,
    };
    uint16_t raw_values[2] = {0xFFFF, 0xFFFF};
    absolute_time_t last_comm_time = 99;

    packet[sizeof(packet) - 1] = usb_calculate_checksum(packet, sizeof(packet) - 1);

    TEST_ASSERT_FALSE(usb_parse_packet(packet, sizeof(packet), raw_values, 2, &last_comm_time));
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, raw_values[0]);
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, raw_values[1]);
    TEST_ASSERT_EQUAL_UINT64(99, last_comm_time);
}

static void test_usb_parse_packet_rejects_bad_checksum(void) {
    uint8_t packet[USB_INPUT_PACKET_SIZE(2)] = {
        USB_INPUT_START_BYTE, 0xE8, 0x03, 0xD0, 0x07, 0xFF,
    };
    uint16_t raw_values[2] = {0};
    absolute_time_t last_comm_time = 123;

    TEST_ASSERT_FALSE(usb_parse_packet(packet, sizeof(packet), raw_values, 2, &last_comm_time));
    TEST_ASSERT_EQUAL_UINT16(0, raw_values[0]);
    TEST_ASSERT_EQUAL_UINT16(0, raw_values[1]);
    TEST_ASSERT_EQUAL_UINT64(123, last_comm_time);
}

static void test_usb_reader_expires_truncated_packet_and_accepts_next_packet(void) {
    uint8_t command[4] = {0};
    uint8_t config[3] = {0};
    usb_packet_reader_t readers[] = {
        {
            .start_byte = 0xA1,
            .buffer = command,
            .packet_size = sizeof(command),
            .kind = USB_PACKET_COMMAND,
        },
        {
            .start_byte = 0xB2,
            .buffer = config,
            .packet_size = sizeof(config),
            .kind = USB_PACKET_CONFIG,
        },
    };

    TEST_ASSERT_EQUAL(USB_PACKET_NONE, usb_process_byte(readers, 2, 0xA1, 1));
    TEST_ASSERT_EQUAL(USB_PACKET_NONE, usb_process_byte(readers, 2, 0x55, 2));
    TEST_ASSERT_EQUAL_UINT(2, readers[0].index);

    absolute_time_t after_timeout = (USB_PACKET_TIMEOUT_MS * 1000u) + 3u;
    TEST_ASSERT_EQUAL(USB_PACKET_NONE, usb_process_byte(readers, 2, 0xB2, after_timeout));
    TEST_ASSERT_EQUAL_UINT(0, readers[0].index);
    TEST_ASSERT_EQUAL_UINT(1, readers[1].index);
    TEST_ASSERT_EQUAL(USB_PACKET_NONE, usb_process_byte(readers, 2, 0x11, after_timeout + 1));
    TEST_ASSERT_EQUAL(USB_PACKET_CONFIG, usb_process_byte(readers, 2, 0x22, after_timeout + 2));
    TEST_ASSERT_EQUAL_UINT(0, readers[1].index);
    TEST_ASSERT_EQUAL_HEX8(0xB2, config[0]);
    TEST_ASSERT_EQUAL_HEX8(0x11, config[1]);
    TEST_ASSERT_EQUAL_HEX8(0x22, config[2]);
}

void test_usb_comm(void) {
    RUN_TEST(test_usb_calculate_checksum_returns_zero_for_empty_data);
    RUN_TEST(test_usb_calculate_checksum_returns_single_byte_value);
    RUN_TEST(test_usb_calculate_checksum_applies_xor_across_known_sequence);
    RUN_TEST(test_usb_calculate_checksum_matches_known_multi_byte_result);
    RUN_TEST(test_usb_parse_packet_accepts_valid_packet_and_extracts_values);
    RUN_TEST(test_usb_parse_packet_rejects_wrong_start_byte);
    RUN_TEST(test_usb_parse_packet_rejects_bad_checksum);
    RUN_TEST(test_usb_reader_expires_truncated_packet_and_accepts_next_packet);
}
