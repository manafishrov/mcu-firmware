#include "usb_tx.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef PICO_ON_DEVICE
#include <class/cdc/cdc_device.h>
#include <hardware/sync.h>

#define TX_SLOTS 8u
#define TX_MAX_PACKET 786u

typedef struct {
    uint16_t length;
    uint8_t data[TX_MAX_PACKET];
} tx_packet_t;
typedef struct {
    tx_packet_t slots[TX_SLOTS];
    uint32_t read;
    uint32_t write;
} tx_queue_t;
static tx_queue_t urgent, ordinary;
static tx_packet_t active;
static uint16_t active_offset;
static uint32_t dropped;

void usb_tx_service(void) {
    /* TinyUSB's USB worker runs on this core. Do not let its IRQ preempt a
       FIFO operation; no wait, USB task, or formatting occurs in this region. */
    uint32_t irq = save_and_disable_interrupts();
    if (!tud_cdc_connected()) {
        active.length = 0;
        active_offset = 0;
        ordinary.read = ordinary.write;
        urgent.read = urgent.write;
        restore_interrupts(irq);
        return;
    }
    if (active.length == 0) {
        tx_queue_t *queue = urgent.read != urgent.write ? &urgent : &ordinary;
        if (queue->read == queue->write) {
            restore_interrupts(irq);
            return;
        }
        active = queue->slots[queue->read % TX_SLOTS];
        queue->read++;
        active_offset = 0;
    }
    uint32_t available = tud_cdc_write_available();
    uint32_t remaining = (uint32_t)active.length - active_offset;
    uint32_t count = remaining < available ? remaining : available;
    if (count > 64u) {
        count = 64u;
    }
    if (count > 0) {
        active_offset += (uint16_t)tud_cdc_write(active.data + active_offset, count);
        (void)tud_cdc_write_flush();
        if (active_offset == active.length) {
            active.length = 0;
        }
    }
    restore_interrupts(irq);
}

bool usb_tx_packet(const uint8_t *data, size_t length, bool priority) {
    usb_tx_service();
    tx_queue_t *queue = priority ? &urgent : &ordinary;
    if (length == 0 || length > TX_MAX_PACKET || queue->write - queue->read == TX_SLOTS) {
        dropped++;
        return false;
    }
    tx_packet_t *slot = &queue->slots[queue->write % TX_SLOTS];
    slot->length = (uint16_t)length;
    memcpy(slot->data, data, length);
    queue->write++;
    usb_tx_service();
    return true;
}

void usb_tx_discard_telemetry(void) {
    ordinary.read = ordinary.write;
}
uint32_t usb_tx_dropped(void) {
    return dropped;
}
#else
#include <stdio.h>
bool usb_tx_packet(const uint8_t *data, size_t length, bool priority) {
    (void)priority;
    bool ok = fwrite(data, 1, length, stdout) == length;
    (void)fflush(stdout);
    return ok;
}
void usb_tx_service(void) {}
void usb_tx_discard_telemetry(void) {}
uint32_t usb_tx_dropped(void) {
    return 0;
}
#endif
