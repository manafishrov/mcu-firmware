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

#define USB_CONFIG_START_BYTE 0xC5
#define USB_CONFIG_PACKET_SIZE 5
#define USB_RUNTIME_CONFIG_STATUS_START_BYTE 0xD5
#define USB_RUNTIME_CONFIG_STATUS_PACKET_SIZE 5
#define USB_RELEASE_VERSION_START_BYTE 0xD6
#define USB_RELEASE_VERSION_MAX_LENGTH 48
#define USB_RELEASE_VERSION_PACKET_OVERHEAD 3

bool mcu_runtime_config_parse_packet(const uint8_t *packet, size_t packet_size,
                                     mcu_runtime_config_t *out_config);
uint16_t mcu_runtime_config_normalize_dshot_speed(uint16_t requested_speed);
void mcu_runtime_config_validate(mcu_runtime_config_t *config);
bool mcu_runtime_config_requires_detector_reset(const mcu_runtime_config_t *current,
                                                const mcu_runtime_config_t *next);
const char *mcu_runtime_config_protocol_name(thruster_protocol_t protocol);
size_t mcu_runtime_config_build_release_packet(uint8_t *packet, size_t packet_capacity);
size_t mcu_runtime_config_build_status_packet(uint8_t *packet, size_t packet_capacity,
                                              const mcu_runtime_config_t *config);
void mcu_runtime_config_send_status(const mcu_runtime_config_t *config);

#endif
