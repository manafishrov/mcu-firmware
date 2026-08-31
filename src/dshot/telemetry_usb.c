#include "telemetry_usb.h"
#include "../usb_comm.h"
#include "dshot.h"
#include "motors.h"
#include <hardware/sync.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TELEMETRY_QUEUE_CAPACITY 256
#define TELEMETRY_FLUSH_BATCH_PACKETS 16
#define TELEMETRY_BATCH_HEADER_SIZE 2
#define TELEMETRY_BATCH_FOOTER_SIZE 1
#define ESC_VERSION_SIGNATURE 0xA5u

typedef struct {
    uint8_t motor_id;
    uint8_t type;
    int32_t value;
} telemetry_queue_entry_t;

static telemetry_queue_entry_t telemetry_queue[TELEMETRY_QUEUE_CAPACITY];
static uint16_t telemetry_queue_head = 0;
static uint16_t telemetry_queue_tail = 0;
static esc_version_decoder_t esc_version_decoders[NUM_MOTORS];
static bool esc_versions_reported[NUM_MOTORS];

static uint8_t esc_version_crc8_update(uint8_t crc, uint8_t value) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x80u) != 0 ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t esc_version_crc8(const char *version, uint8_t length) {
    uint8_t crc = esc_version_crc8_update(0, length);
    for (uint8_t i = 0; i < length; ++i) {
        crc = esc_version_crc8_update(crc, (uint8_t)version[i]);
    }
    return crc;
}

static uint16_t telemetry_queue_advance(uint16_t index) {
    return (uint16_t)((index + 1) % TELEMETRY_QUEUE_CAPACITY);
}

void dshot_telemetry_usb_init(void) {
    telemetry_queue_head = 0;
    telemetry_queue_tail = 0;
    memset(esc_version_decoders, 0, sizeof(esc_version_decoders));
    memset(esc_versions_reported, 0, sizeof(esc_versions_reported));
}

void dshot_telemetry_usb_reset(void) {
    telemetry_queue_head = 0;
    telemetry_queue_tail = 0;
    memset(esc_version_decoders, 0, sizeof(esc_version_decoders));
    memset(esc_versions_reported, 0, sizeof(esc_versions_reported));
}

void dshot_telemetry_usb_begin_esc_version_discovery(void) {
    memset(esc_version_decoders, 0, sizeof(esc_version_decoders));
    memset(esc_versions_reported, 0, sizeof(esc_versions_reported));
}

bool dshot_telemetry_usb_all_esc_versions_reported(void) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (!esc_versions_reported[i]) {
            return false;
        }
    }
    return true;
}

uint8_t dshot_telemetry_usb_esc_versions_reported_count(void) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (esc_versions_reported[i]) {
            count++;
        }
    }
    return count;
}

void dshot_telemetry_usb_send_discovery_complete(void) {
    dshot_telemetry_usb_send(UINT8_MAX, TELEMETRY_TYPE_ESC_VERSION_DISCOVERY_COMPLETE,
                             dshot_telemetry_usb_esc_versions_reported_count());
}

void dshot_telemetry_usb_flush(void) {
    if (telemetry_queue_tail == telemetry_queue_head) {
        return;
    }

    while (true) {
        uint8_t batch[TELEMETRY_BATCH_HEADER_SIZE +
                      (TELEMETRY_FLUSH_BATCH_PACKETS * TELEMETRY_BATCH_ENTRY_SIZE) +
                      TELEMETRY_BATCH_FOOTER_SIZE];
        size_t batch_len = 0;
        uint32_t irq_state = save_and_disable_interrupts();

        if (telemetry_queue_tail == telemetry_queue_head) {
            restore_interrupts(irq_state);
            break;
        }

        batch[0] = TELEMETRY_BATCH_START_BYTE;
        batch[1] = 0;
        batch_len = TELEMETRY_BATCH_HEADER_SIZE;

        while (telemetry_queue_tail != telemetry_queue_head &&
               batch[1] < TELEMETRY_FLUSH_BATCH_PACKETS &&
               batch_len + TELEMETRY_BATCH_ENTRY_SIZE + TELEMETRY_BATCH_FOOTER_SIZE <=
                   sizeof(batch)) {
            const telemetry_queue_entry_t *entry = &telemetry_queue[telemetry_queue_tail];
            batch[batch_len] = entry->motor_id;
            batch[batch_len + 1] = entry->type;
            memcpy(&batch[batch_len + 2], &entry->value, sizeof(entry->value));
            batch_len += TELEMETRY_BATCH_ENTRY_SIZE;
            batch[1]++;
            telemetry_queue_tail = telemetry_queue_advance(telemetry_queue_tail);
        }

        restore_interrupts(irq_state);

        batch[batch_len] = usb_calculate_checksum(batch, batch_len);
        batch_len += TELEMETRY_BATCH_FOOTER_SIZE;
        fwrite(batch, 1, batch_len, stdout);
    }

    fflush(stdout);
}

void dshot_telemetry_usb_send(uint8_t motor_id, uint8_t type, int32_t value) {
    uint32_t irq_state = save_and_disable_interrupts();
    uint16_t next_head = telemetry_queue_advance(telemetry_queue_head);

    if (next_head == telemetry_queue_tail) {
        telemetry_queue_tail = telemetry_queue_advance(telemetry_queue_tail);
    }

    telemetry_queue_entry_t *entry = &telemetry_queue[telemetry_queue_head];
    entry->motor_id = motor_id;
    entry->type = type;
    entry->value = value;
    telemetry_queue_head = next_head;
    restore_interrupts(irq_state);
}

bool dshot_telemetry_usb_decode_esc_version(esc_version_decoder_t *decoder,
                                            enum dshot_telemetry_type type, uint32_t value,
                                            char version[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1]) {
    if (type == DSHOT_TELEMETRY_TYPE_DEBUG1 && value == ESC_VERSION_SIGNATURE) {
        decoder->stage = 1;
        decoder->length = 0;
        decoder->received = 0;
        return false;
    }
    if (type == DSHOT_TELEMETRY_TYPE_DEBUG2 && decoder->stage == 1 && value > 0 &&
        value <= ESC_FIRMWARE_VERSION_MAX_LENGTH) {
        decoder->length = (uint8_t)value;
        decoder->stage = 2;
        return false;
    }
    if (type == DSHOT_TELEMETRY_TYPE_DEBUG3 && decoder->stage == 2) {
        if (value == 0 || value > UINT8_MAX) {
            decoder->stage = 0;
            return false;
        }
        decoder->value[decoder->received++] = (char)value;
        if (decoder->received == decoder->length) {
            decoder->stage = 3;
        }
        return false;
    }
    if (type == DSHOT_TELEMETRY_TYPE_DEBUG2 && decoder->stage == 3 &&
        value == esc_version_crc8(decoder->value, decoder->length)) {
        decoder->value[decoder->length] = '\0';
        memcpy(version, decoder->value, (size_t)decoder->length + 1);
        decoder->stage = 0;
        return true;
    }
    if (type == DSHOT_TELEMETRY_TYPE_DEBUG1 || type == DSHOT_TELEMETRY_TYPE_DEBUG2 ||
        type == DSHOT_TELEMETRY_TYPE_DEBUG3) {
        decoder->stage = 0;
    }
    return false;
}

static void dshot_telemetry_usb_send_esc_version(uint8_t motor_id, const char *version) {
    uint8_t length = (uint8_t)strlen(version);
    dshot_telemetry_usb_send(motor_id, TELEMETRY_TYPE_ESC_VERSION_LENGTH, length);
    for (uint8_t offset = 0; offset < length; offset += 3) {
        uint32_t packed = (uint32_t)(offset / 3) << 24;
        for (uint8_t i = 0; i < 3 && offset + i < length; ++i) {
            packed |= (uint32_t)(uint8_t)version[offset + i] << (i * 8);
        }
        dshot_telemetry_usb_send(motor_id, TELEMETRY_TYPE_ESC_VERSION_CHUNK, (int32_t)packed);
    }
    dshot_telemetry_usb_send(motor_id, TELEMETRY_TYPE_ESC_VERSION_COMPLETE,
                             esc_version_crc8(version, length));
}

void dshot_telemetry_callback(void *context, int channel, enum dshot_telemetry_type type,
                              uint32_t value) {
    dshot_telemetry_context_t *ctx = (dshot_telemetry_context_t *)context;
    uint8_t global_motor_id = ctx->controller_base_global_id + channel;

    switch (type) {
    case DSHOT_TELEMETRY_TYPE_ERPM:
        dshot_telemetry_usb_send(global_motor_id, TELEMETRY_TYPE_ERPM, (int32_t)value);
        break;
    case DSHOT_TELEMETRY_TYPE_VOLTAGE: {
        dshot_telemetry_usb_send(global_motor_id, TELEMETRY_TYPE_VOLTAGE, (int32_t)value);
        break;
    }
    case DSHOT_TELEMETRY_TYPE_TEMPERATURE: {
        dshot_telemetry_usb_send(global_motor_id, TELEMETRY_TYPE_TEMPERATURE, (int32_t)value);
        break;
    }
    case DSHOT_TELEMETRY_TYPE_CURRENT: {
        dshot_telemetry_usb_send(global_motor_id, TELEMETRY_TYPE_CURRENT, (int32_t)value);
        break;
    }
    case DSHOT_TELEMETRY_TYPE_DEBUG1:
    case DSHOT_TELEMETRY_TYPE_DEBUG2:
    case DSHOT_TELEMETRY_TYPE_DEBUG3: {
        char version[ESC_FIRMWARE_VERSION_MAX_LENGTH + 1];
        if (dshot_telemetry_usb_decode_esc_version(&esc_version_decoders[global_motor_id], type,
                                                   value, version)) {
            esc_versions_reported[global_motor_id] = true;
            dshot_telemetry_usb_send_esc_version(global_motor_id, version);
        }
        break;
    }
    default:
        break;
    }
}
