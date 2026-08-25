#ifndef DSHOT_TELEMETRY_USB_H
#define DSHOT_TELEMETRY_USB_H

#include "dshot.h"
#include <stdint.h>

#define TELEMETRY_START_BYTE 0xA5
#define TELEMETRY_PACKET_SIZE 8
#define TELEMETRY_BATCH_START_BYTE 0xA6
#define TELEMETRY_BATCH_ENTRY_SIZE 6
#define TELEMETRY_TYPE_ERPM 0
#define TELEMETRY_TYPE_VOLTAGE 1
#define TELEMETRY_TYPE_TEMPERATURE 2
#define TELEMETRY_TYPE_CURRENT 3
#define TELEMETRY_TYPE_SIGNAL_QUALITY 4
#define TELEMETRY_SIGNAL_QUALITY_UNAVAILABLE -1
#define TELEMETRY_TYPE_ESC_VERSION_LENGTH 5
#define TELEMETRY_TYPE_ESC_VERSION_CHUNK 6
#define TELEMETRY_TYPE_ESC_VERSION_COMPLETE 7
#define ESC_FIRMWARE_VERSION_MAX_LENGTH 31

typedef struct {
    uint8_t controller_base_global_id;
} dshot_telemetry_context_t;

typedef struct {
    uint8_t stage;
    uint8_t length;
    uint8_t received;
    char value[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1];
} esc_version_decoder_t;

void dshot_telemetry_usb_init(void);
void dshot_telemetry_usb_reset(void);
void dshot_telemetry_usb_begin_esc_version_discovery(void);
bool dshot_telemetry_usb_all_esc_versions_reported(void);
uint8_t dshot_telemetry_usb_esc_versions_reported_count(void);
void dshot_telemetry_usb_send(uint8_t motor_id, uint8_t type, int32_t value);
void dshot_telemetry_usb_flush(void);
bool dshot_telemetry_usb_decode_esc_version(esc_version_decoder_t *decoder,
                                            enum dshot_telemetry_type type, uint32_t value,
                                            char version[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1]);
void dshot_telemetry_callback(void *context, int channel, enum dshot_telemetry_type type,
                              uint32_t value);

#endif
