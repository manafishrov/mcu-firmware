#include "esc_firmware/update.h"
#include "unity/unity.h"
#include "usb_comm.h"
#include <stdint.h>
#include <string.h>

static void make_control_packet(uint8_t *packet, esc_firmware_update_command_t command,
                                const uint8_t *image) {
    memset(packet, 0, ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE);
    packet[0] = ESC_FIRMWARE_USB_CONTROL_START_BYTE;
    packet[1] = (uint8_t)command;
    packet[2] = 1;
    if (command == ESC_FIRMWARE_UPDATE_COMMAND_BEGIN ||
        command == ESC_FIRMWARE_UPDATE_COMMAND_RECOVER_BEGIN) {
        packet[3] = (uint8_t)ESC_FIRMWARE_IMAGE_SIZE;
        packet[4] = (uint8_t)(ESC_FIRMWARE_IMAGE_SIZE >> 8);
        uint32_t crc = esc_firmware_update_crc32(image, ESC_FIRMWARE_IMAGE_SIZE);
        packet[5] = (uint8_t)crc;
        packet[6] = (uint8_t)(crc >> 8);
        packet[7] = (uint8_t)(crc >> 16);
        packet[8] = (uint8_t)(crc >> 24);
        packet[9] = ESC_FIRMWARE_TARGET_F421_PB4_32K;
    }
    packet[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE - 1] =
        usb_calculate_checksum(packet, ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE - 1);
}

static void receive_image(const uint8_t *image) {
    uint8_t packet[ESC_FIRMWARE_USB_DATA_PACKET_SIZE];
    uint16_t sequence = 1;
    for (uint16_t offset = 0; offset < ESC_FIRMWARE_IMAGE_SIZE;
         offset += ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE) {
        memset(packet, 0, sizeof(packet));
        packet[0] = ESC_FIRMWARE_USB_DATA_START_BYTE;
        packet[1] = 1;
        packet[2] = (uint8_t)sequence;
        packet[3] = (uint8_t)(sequence >> 8);
        packet[4] = (uint8_t)offset;
        packet[5] = (uint8_t)(offset >> 8);
        uint16_t remaining = ESC_FIRMWARE_IMAGE_SIZE - offset;
        uint8_t length = remaining < ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE
                             ? (uint8_t)remaining
                             : ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE;
        packet[6] = length;
        memcpy(&packet[7], &image[offset], length);
        packet[ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1] =
            usb_calculate_checksum(packet, ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1);
        esc_firmware_update_error_t error;
        TEST_ASSERT_TRUE(esc_firmware_update_receive_data(packet, &error));
        TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_NONE, error);
        sequence++;
    }
}

static void make_valid_image(uint8_t *image) {
    static const char target_name[] = "SKYSTARS_AM60_V2_F421";
    memset(image, 0xFF, ESC_FIRMWARE_IMAGE_SIZE);
    const uint32_t stack_pointer = 0x20001000;
    const uint32_t reset_handler = ESC_FIRMWARE_APPLICATION_ADDRESS + 0x101;
    memcpy(image, &stack_pointer, sizeof(stack_pointer));
    memcpy(&image[4], &reset_handler, sizeof(reset_handler));
    memcpy(&image[ESC_FIRMWARE_IMAGE_SIZE - 32], target_name, sizeof(target_name) - 1);
}

static void test_esc_firmware_crc32_matches_standard_vector(void) {
    const uint8_t value[] = "123456789";
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926, esc_firmware_update_crc32(value, sizeof(value) - 1));
}

static void test_esc_firmware_update_accepts_complete_target_image(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    make_valid_image(image);
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, command);
    receive_image(image);
    TEST_ASSERT_EQUAL_UINT16(ESC_FIRMWARE_IMAGE_SIZE, esc_firmware_update_received_size());
    TEST_ASSERT_TRUE(esc_firmware_update_validate_image(&error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_NONE, error);
}

static void test_esc_firmware_update_rejects_out_of_order_chunk(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    uint8_t data[ESC_FIRMWARE_USB_DATA_PACKET_SIZE] = {0};
    make_valid_image(image);
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    data[0] = ESC_FIRMWARE_USB_DATA_START_BYTE;
    data[1] = 1;
    data[2] = 1;
    data[4] = 128;
    data[6] = ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE;
    data[ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1] =
        usb_calculate_checksum(data, ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1);

    TEST_ASSERT_FALSE(esc_firmware_update_receive_data(data, &error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE, error);
}

static void test_esc_firmware_update_tracks_recovery_transaction(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    make_valid_image(image);
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_RECOVER_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_COMMAND_RECOVER_BEGIN, command);
    TEST_ASSERT_TRUE(esc_firmware_update_recovery_requested());

    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_ABORT, image);
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    TEST_ASSERT_FALSE(esc_firmware_update_recovery_requested());
}

static void test_esc_firmware_update_accepts_duplicate_last_chunk(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    uint8_t data[ESC_FIRMWARE_USB_DATA_PACKET_SIZE] = {0};
    make_valid_image(image);
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    TEST_ASSERT_TRUE(esc_firmware_update_receiving());

    data[0] = ESC_FIRMWARE_USB_DATA_START_BYTE;
    data[1] = 1;
    data[2] = 1;
    data[6] = ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE;
    memcpy(&data[7], image, ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE);
    data[ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1] =
        usb_calculate_checksum(data, ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1);

    TEST_ASSERT_TRUE(esc_firmware_update_receive_data(data, &error));
    TEST_ASSERT_EQUAL_UINT16(ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE,
                             esc_firmware_update_received_size());
    TEST_ASSERT_TRUE(esc_firmware_update_receive_data(data, &error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_NONE, error);
    TEST_ASSERT_EQUAL_UINT16(ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE,
                             esc_firmware_update_received_size());

    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_ABORT, image);
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    TEST_ASSERT_FALSE(esc_firmware_update_receiving());
}

static void test_esc_firmware_update_rejects_partial_repeat_of_last_chunk(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    uint8_t data[ESC_FIRMWARE_USB_DATA_PACKET_SIZE] = {0};
    make_valid_image(image);
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));

    data[0] = ESC_FIRMWARE_USB_DATA_START_BYTE;
    data[1] = 1;
    data[2] = 1;
    data[6] = ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE;
    memcpy(&data[7], image, ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE);
    data[ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1] =
        usb_calculate_checksum(data, ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1);
    TEST_ASSERT_TRUE(esc_firmware_update_receive_data(data, &error));

    const uint16_t partial_offset = ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE / 2;
    data[4] = (uint8_t)partial_offset;
    data[5] = (uint8_t)(partial_offset >> 8);
    data[6] = ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE / 2;
    memcpy(&data[7], &image[partial_offset], data[6]);
    memset(&data[7 + data[6]], 0, ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE - data[6]);
    data[ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1] =
        usb_calculate_checksum(data, ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1);

    TEST_ASSERT_FALSE(esc_firmware_update_receive_data(data, &error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE, error);
}

static void test_esc_firmware_update_rejects_reset_handler_in_eeprom(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    make_valid_image(image);
    const uint32_t reset_handler = ESC_FIRMWARE_EEPROM_ADDRESS + 1;
    memcpy(&image[4], &reset_handler, sizeof(reset_handler));
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    receive_image(image);
    TEST_ASSERT_FALSE(esc_firmware_update_validate_image(&error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_INVALID_IMAGE, error);
}

static void test_esc_firmware_update_rejects_non_thumb_reset_handler(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    make_valid_image(image);
    const uint32_t reset_handler = ESC_FIRMWARE_APPLICATION_ADDRESS + 0x100;
    memcpy(&image[4], &reset_handler, sizeof(reset_handler));
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    receive_image(image);
    TEST_ASSERT_FALSE(esc_firmware_update_validate_image(&error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_INVALID_IMAGE, error);
}

static void test_esc_firmware_update_rejects_image_crc_mismatch(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    make_valid_image(image);
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);
    image[16] ^= 1;

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    receive_image(image);
    TEST_ASSERT_FALSE(esc_firmware_update_validate_image(&error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_IMAGE_CRC, error);
}

static void test_esc_firmware_update_rejects_commit_before_upload_completes(void) {
    static uint8_t image[ESC_FIRMWARE_IMAGE_SIZE];
    uint8_t control[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    make_valid_image(image);
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_BEGIN, image);

    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    TEST_ASSERT_TRUE(esc_firmware_update_parse_control(control, &command, &error));
    make_control_packet(control, ESC_FIRMWARE_UPDATE_COMMAND_COMMIT, image);
    TEST_ASSERT_FALSE(esc_firmware_update_parse_control(control, &command, &error));
    TEST_ASSERT_EQUAL(ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE, error);
}

void test_esc_firmware_update(void) {
    RUN_TEST(test_esc_firmware_crc32_matches_standard_vector);
    RUN_TEST(test_esc_firmware_update_accepts_complete_target_image);
    RUN_TEST(test_esc_firmware_update_rejects_out_of_order_chunk);
    RUN_TEST(test_esc_firmware_update_tracks_recovery_transaction);
    RUN_TEST(test_esc_firmware_update_accepts_duplicate_last_chunk);
    RUN_TEST(test_esc_firmware_update_rejects_partial_repeat_of_last_chunk);
    RUN_TEST(test_esc_firmware_update_rejects_reset_handler_in_eeprom);
    RUN_TEST(test_esc_firmware_update_rejects_non_thumb_reset_handler);
    RUN_TEST(test_esc_firmware_update_rejects_image_crc_mismatch);
    RUN_TEST(test_esc_firmware_update_rejects_commit_before_upload_completes);
}
