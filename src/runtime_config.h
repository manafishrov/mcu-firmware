#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    THRUSTER_PROTOCOL_PWM = 0,
    THRUSTER_PROTOCOL_DSHOT = 1,
} thruster_protocol_t;

typedef struct {
    thruster_protocol_t protocol;
    uint16_t dshot_speed;
} mcu_runtime_config_t;

typedef enum {
    MCU_CONTROL_COMMAND_APPLY_CONFIG = 1,
    MCU_CONTROL_COMMAND_GET_INFO = 2,
} mcu_control_command_t;

typedef struct {
    mcu_control_command_t command;
    uint8_t request_id;
    mcu_runtime_config_t config;
} mcu_control_request_t;

typedef enum {
    MCU_RUNTIME_CONFIG_STATE_APPLYING = 1,
    MCU_RUNTIME_CONFIG_STATE_APPLIED = 2,
    MCU_RUNTIME_CONFIG_STATE_REJECTED = 3,
} mcu_runtime_config_state_t;

typedef enum {
    MCU_RUNTIME_CONFIG_ERROR_NONE = 0,
    MCU_RUNTIME_CONFIG_ERROR_THRUSTERS_ACTIVE = 1,
    MCU_RUNTIME_CONFIG_ERROR_ESC_RECOVERY_REQUIRED = 2,
    MCU_RUNTIME_CONFIG_ERROR_APPLY_IN_PROGRESS = 3,
} mcu_runtime_config_error_t;

#define USB_CONFIG_START_BYTE 0xC5
#define USB_CONFIG_PACKET_SIZE 7
#define USB_RUNTIME_CONFIG_STATUS_START_BYTE 0xD5
#define USB_RUNTIME_CONFIG_STATUS_PACKET_SIZE 8
#define USB_RELEASE_VERSION_START_BYTE 0xD6
#define USB_RELEASE_VERSION_MAX_LENGTH 48
#define USB_RELEASE_VERSION_PACKET_OVERHEAD 4

bool mcu_runtime_config_parse_packet(const uint8_t *packet, size_t packet_size,
                                     mcu_control_request_t *out_request);
uint16_t mcu_runtime_config_normalize_dshot_speed(uint16_t requested_speed);
void mcu_runtime_config_validate(mcu_runtime_config_t *config);
bool mcu_runtime_config_requires_detector_reset(const mcu_runtime_config_t *current,
                                                const mcu_runtime_config_t *next);
const char *mcu_runtime_config_protocol_name(thruster_protocol_t protocol);
size_t mcu_runtime_config_build_release_packet(uint8_t *packet, size_t packet_capacity,
                                               uint8_t request_id);
size_t mcu_runtime_config_build_status_packet(uint8_t *packet, size_t packet_capacity,
                                              uint8_t request_id, mcu_runtime_config_state_t state,
                                              mcu_runtime_config_error_t error,
                                              const mcu_runtime_config_t *config);
void mcu_runtime_config_send_release(uint8_t request_id);
void mcu_runtime_config_send_status(uint8_t request_id, mcu_runtime_config_state_t state,
                                    mcu_runtime_config_error_t error,
                                    const mcu_runtime_config_t *config);

#endif
