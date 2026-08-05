#include "update.h"
#include "usb_comm.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ESC_FIRMWARE_SRAM_START 0x20000000u
#define ESC_FIRMWARE_SRAM_END 0x20010000u
#define ESC_FIRMWARE_FILE_NAME_SIZE 32

static const char expected_file_name[] = "SKYSTARS_AM60_V2_F421";
static uint8_t image_buffer[ESC_FIRMWARE_IMAGE_SIZE];
static uint16_t expected_size;
static uint16_t received_size;
static uint32_t expected_crc32;
static bool receiving;

static uint16_t read_le16(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_le32(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void write_le32(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

void esc_firmware_update_reset(void) {
    expected_size = 0;
    received_size = 0;
    expected_crc32 = 0;
    receiving = false;
}

bool esc_firmware_update_parse_control(const uint8_t *packet,
                                       esc_firmware_update_command_t *command,
                                       esc_firmware_update_error_t *error) {
    if (packet[0] != ESC_FIRMWARE_USB_CONTROL_START_BYTE ||
        usb_calculate_checksum(packet, ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE - 1) !=
            packet[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE - 1]) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_BAD_PACKET;
        return false;
    }

    *command = (esc_firmware_update_command_t)packet[1];
    if (*command == ESC_FIRMWARE_UPDATE_COMMAND_ABORT) {
        esc_firmware_update_reset();
        *error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
        return true;
    }
    if (*command == ESC_FIRMWARE_UPDATE_COMMAND_COMMIT) {
        if (!receiving || received_size != expected_size) {
            *error = ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE;
            return false;
        }
        *error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
        return true;
    }
    if (*command != ESC_FIRMWARE_UPDATE_COMMAND_BEGIN) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_BAD_PACKET;
        return false;
    }

    uint16_t size = read_le16(&packet[2]);
    if (packet[8] != ESC_FIRMWARE_TARGET_F421_PB4_32K || packet[9] != 0 || packet[10] != 0 ||
        size != ESC_FIRMWARE_IMAGE_SIZE) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_INVALID_IMAGE;
        return false;
    }

    expected_size = size;
    received_size = 0;
    expected_crc32 = read_le32(&packet[4]);
    receiving = true;
    *error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
    return true;
}

bool esc_firmware_update_receive_data(const uint8_t *packet, esc_firmware_update_error_t *error) {
    if (packet[0] != ESC_FIRMWARE_USB_DATA_START_BYTE ||
        usb_calculate_checksum(packet, ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1) !=
            packet[ESC_FIRMWARE_USB_DATA_PACKET_SIZE - 1]) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_BAD_PACKET;
        return false;
    }
    if (!receiving) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE;
        return false;
    }

    uint16_t offset = read_le16(&packet[1]);
    uint8_t length = packet[3];
    if (length == 0 || length > ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE || offset != received_size ||
        (uint32_t)offset + length > expected_size) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE;
        return false;
    }

    memcpy(&image_buffer[offset], &packet[4], length);
    received_size += length;
    *error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
    return true;
}

uint32_t esc_firmware_update_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

bool esc_firmware_update_validate_image(esc_firmware_update_error_t *error) {
    if (!receiving || received_size != ESC_FIRMWARE_IMAGE_SIZE) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE;
        return false;
    }
    if (esc_firmware_update_crc32(image_buffer, received_size) != expected_crc32) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_IMAGE_CRC;
        return false;
    }

    uint32_t stack_pointer = read_le32(image_buffer);
    uint32_t reset_handler = read_le32(&image_buffer[4]);
    size_t file_name_offset = ESC_FIRMWARE_IMAGE_SIZE - ESC_FIRMWARE_FILE_NAME_SIZE;
    if (stack_pointer < ESC_FIRMWARE_SRAM_START || stack_pointer > ESC_FIRMWARE_SRAM_END ||
        reset_handler < ESC_FIRMWARE_APPLICATION_ADDRESS ||
        reset_handler >= ESC_FIRMWARE_EEPROM_ADDRESS || (reset_handler & 1u) == 0 ||
        memcmp(&image_buffer[file_name_offset], expected_file_name,
               sizeof(expected_file_name) - 1) != 0) {
        *error = ESC_FIRMWARE_UPDATE_ERROR_INVALID_IMAGE;
        return false;
    }

    *error = ESC_FIRMWARE_UPDATE_ERROR_NONE;
    return true;
}

const uint8_t *esc_firmware_update_image(void) {
    return image_buffer;
}

uint16_t esc_firmware_update_image_size(void) {
    return expected_size;
}

uint16_t esc_firmware_update_received_size(void) {
    return received_size;
}

void esc_firmware_update_send_status(esc_firmware_update_status_t status, uint8_t motor,
                                     esc_firmware_update_error_t error, uint32_t value) {
    uint8_t packet[ESC_FIRMWARE_USB_STATUS_PACKET_SIZE] = {
        ESC_FIRMWARE_USB_STATUS_START_BYTE,
        (uint8_t)status,
        motor,
        (uint8_t)error,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    write_le32(&packet[4], value);
    packet[ESC_FIRMWARE_USB_STATUS_PACKET_SIZE - 1] =
        usb_calculate_checksum(packet, ESC_FIRMWARE_USB_STATUS_PACKET_SIZE - 1);
    fwrite(packet, 1, sizeof(packet), stdout);
}
