#include "runtime_config.h"
#include "usb_comm.h"
#include "version.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static bool mcu_supports_dshot_1200(void) {
#if defined(PICO_RP2350)
    return true;
#else
    return false;
#endif
}

uint16_t mcu_runtime_config_normalize_dshot_speed(uint16_t requested_speed) {
    switch (requested_speed) {
    case 150:
    case 300:
    case 600:
        return requested_speed;
    case 1200:
        return mcu_supports_dshot_1200() ? requested_speed : 600;
    default:
        return 300;
    }
}

void mcu_runtime_config_validate(mcu_runtime_config_t *config) {
    config->dshot_speed = mcu_runtime_config_normalize_dshot_speed(config->dshot_speed);
    if (config->protocol != THRUSTER_PROTOCOL_PWM && config->protocol != THRUSTER_PROTOCOL_DSHOT) {
        config->protocol = THRUSTER_PROTOCOL_DSHOT;
    }
}

bool mcu_runtime_config_requires_detector_reset(const mcu_runtime_config_t *current,
                                                const mcu_runtime_config_t *next) {
    if (current->protocol != next->protocol) {
        return true;
    }

    return current->protocol == THRUSTER_PROTOCOL_DSHOT &&
           current->dshot_speed != next->dshot_speed;
}

bool mcu_runtime_config_parse_packet(const uint8_t *packet, size_t packet_size,
                                     mcu_runtime_config_t *out_config) {
    if (packet_size != USB_CONFIG_PACKET_SIZE || packet[0] != USB_CONFIG_START_BYTE) {
        return false;
    }

    uint8_t received_checksum = packet[packet_size - 1];
    uint8_t calculated_checksum = usb_calculate_checksum(packet, packet_size - 1);
    if (received_checksum != calculated_checksum) {
        return false;
    }

    out_config->protocol = (thruster_protocol_t)packet[1];
    out_config->dshot_speed = (uint16_t)packet[2] | ((uint16_t)packet[3] << 8);
    mcu_runtime_config_validate(out_config);
    return true;
}

const char *mcu_runtime_config_protocol_name(thruster_protocol_t protocol) {
    return protocol == THRUSTER_PROTOCOL_PWM ? "PWM" : "DShot";
}

void mcu_runtime_config_send_version(const mcu_runtime_config_t *config) {
    uint8_t packet[USB_VERSION_PACKET_SIZE];
    packet[0] = USB_VERSION_START_BYTE;
    packet[1] = MCU_FIRMWARE_VERSION_MAJOR;
    packet[2] = MCU_FIRMWARE_VERSION_MINOR;
    packet[3] = MCU_FIRMWARE_VERSION_PATCH;
    packet[4] = (uint8_t)config->protocol;
    packet[5] = (uint8_t)(config->dshot_speed & 0xFF);
    packet[6] = (uint8_t)(config->dshot_speed >> 8);
    packet[7] = usb_calculate_checksum(packet, USB_VERSION_PACKET_SIZE - 1);
    fwrite(packet, 1, USB_VERSION_PACKET_SIZE, stdout);
    fflush(stdout);
}
