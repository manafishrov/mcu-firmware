#ifndef ESC_FIRMWARE_UPDATE_H
#define ESC_FIRMWARE_UPDATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ESC_FIRMWARE_USB_CONTROL_START_BYTE 0xE7
#define ESC_FIRMWARE_USB_DATA_START_BYTE 0xE8
#define ESC_FIRMWARE_USB_STATUS_START_BYTE 0xE9

#define ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE 12
#define ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE 128
#define ESC_FIRMWARE_USB_DATA_PACKET_SIZE (1 + 2 + 1 + ESC_FIRMWARE_USB_DATA_PAYLOAD_SIZE + 1)
#define ESC_FIRMWARE_USB_STATUS_PACKET_SIZE 12

#define ESC_FIRMWARE_APPLICATION_ADDRESS 0x08001000u
#define ESC_FIRMWARE_EEPROM_ADDRESS 0x08007C00u
#define ESC_FIRMWARE_IMAGE_SIZE (ESC_FIRMWARE_EEPROM_ADDRESS - ESC_FIRMWARE_APPLICATION_ADDRESS)
#define ESC_FIRMWARE_TARGET_F421_PB4_32K 1

typedef enum {
    ESC_FIRMWARE_UPDATE_COMMAND_BEGIN = 1,
    ESC_FIRMWARE_UPDATE_COMMAND_COMMIT = 2,
    ESC_FIRMWARE_UPDATE_COMMAND_ABORT = 3,
    ESC_FIRMWARE_UPDATE_COMMAND_RECOVER_BEGIN = 4,
} esc_firmware_update_command_t;

typedef enum {
    ESC_FIRMWARE_UPDATE_STATUS_READY = 1,
    ESC_FIRMWARE_UPDATE_STATUS_RECEIVED = 2,
    ESC_FIRMWARE_UPDATE_STATUS_ENTERING_BOOTLOADER = 3,
    ESC_FIRMWARE_UPDATE_STATUS_MOTOR_BEGIN = 4,
    ESC_FIRMWARE_UPDATE_STATUS_MOTOR_DONE = 5,
    ESC_FIRMWARE_UPDATE_STATUS_COMPLETE = 6,
    ESC_FIRMWARE_UPDATE_STATUS_FAILED = 7,
    ESC_FIRMWARE_UPDATE_STATUS_ABORTED = 8,
    ESC_FIRMWARE_UPDATE_STATUS_RECOVERY_REQUIRED = 9,
} esc_firmware_update_status_t;

typedef enum {
    ESC_FIRMWARE_UPDATE_ERROR_NONE = 0,
    ESC_FIRMWARE_UPDATE_ERROR_BAD_PACKET = 1,
    ESC_FIRMWARE_UPDATE_ERROR_NOT_DSHOT = 2,
    ESC_FIRMWARE_UPDATE_ERROR_THRUSTERS_ACTIVE = 3,
    ESC_FIRMWARE_UPDATE_ERROR_INVALID_IMAGE = 4,
    ESC_FIRMWARE_UPDATE_ERROR_BAD_SEQUENCE = 5,
    ESC_FIRMWARE_UPDATE_ERROR_IMAGE_CRC = 6,
    ESC_FIRMWARE_UPDATE_ERROR_BOOTLOADER_CONNECT = 7,
    ESC_FIRMWARE_UPDATE_ERROR_WRONG_TARGET = 8,
    ESC_FIRMWARE_UPDATE_ERROR_PROGRAM = 9,
    ESC_FIRMWARE_UPDATE_ERROR_VERIFY = 10,
} esc_firmware_update_error_t;

void esc_firmware_update_reset(void);
bool esc_firmware_update_parse_control(const uint8_t *packet,
                                       esc_firmware_update_command_t *command,
                                       esc_firmware_update_error_t *error);
bool esc_firmware_update_receive_data(const uint8_t *packet, esc_firmware_update_error_t *error);
bool esc_firmware_update_receiving(void);
bool esc_firmware_update_recovery_requested(void);
bool esc_firmware_update_validate_image(esc_firmware_update_error_t *error);
const uint8_t *esc_firmware_update_image(void);
uint16_t esc_firmware_update_image_size(void);
uint16_t esc_firmware_update_received_size(void);
uint32_t esc_firmware_update_crc32(const uint8_t *data, size_t length);
void esc_firmware_update_send_status(esc_firmware_update_status_t status, uint8_t motor,
                                     esc_firmware_update_error_t error, uint32_t value);

#endif
