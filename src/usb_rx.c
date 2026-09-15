#include "usb_rx.h"
#include <pico/error.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef PICO_ON_DEVICE
#include <class/cdc/cdc_device.h>
#include <hardware/sync.h>
#include <pico/types.h>

#define RX_SLOTS 32u
#define RX_CHUNK_SIZE 64u

typedef struct {
    absolute_time_t arrival;
    uint8_t data[RX_CHUNK_SIZE];
    uint16_t length;
} rx_chunk_t;
static rx_chunk_t chunks[RX_SLOTS];
static volatile uint32_t read_index, write_index;
static volatile bool overflow;
static uint16_t read_offset;

static void receive_available(void *context) {
    (void)context;
    /* Called by the SDK USB worker after releasing its stdio mutex. Bounded
       reads only; no parsing, control, logging, or allocations in this IRQ. */
    absolute_time_t arrival = get_absolute_time();
    for (unsigned budget = 0; budget < 8 && tud_cdc_available() > 0; ++budget) {
        uint32_t write = write_index;
        if (write - read_index == RX_SLOTS) {
            uint8_t discarded[RX_CHUNK_SIZE];
            (void)tud_cdc_read(discarded, sizeof(discarded));
            overflow = true;
            continue;
        }
        rx_chunk_t *chunk = &chunks[write % RX_SLOTS];
        chunk->length = (uint16_t)tud_cdc_read(chunk->data, sizeof(chunk->data));
        if (chunk->length == 0) {
            break;
        }
        chunk->arrival = arrival;
        __dmb();
        write_index = write + 1u;
    }
}

void usb_rx_init(void) {
    stdio_set_chars_available_callback(receive_available, NULL);
}

int usb_rx_get(absolute_time_t *arrival) {
    uint32_t read = read_index;
    if (read == write_index) {
        return PICO_ERROR_TIMEOUT;
    }
    __dmb();
    const rx_chunk_t *chunk = &chunks[read % RX_SLOTS];
    *arrival = chunk->arrival;
    int byte = chunk->data[read_offset++];
    if (read_offset == chunk->length) {
        read_offset = 0;
        __dmb();
        read_index = read + 1u;
    }
    return byte;
}

bool usb_rx_take_overflow(void) {
    uint32_t irq = save_and_disable_interrupts();
    bool had_overflow = overflow;
    if (had_overflow) {
        read_index = write_index;
        read_offset = 0;
        overflow = false;
    }
    restore_interrupts(irq);
    return had_overflow;
}
#else
void usb_rx_init(void) {}
int usb_rx_get(absolute_time_t *arrival) {
    *arrival = get_absolute_time();
    return getchar_timeout_us(0);
}
bool usb_rx_take_overflow(void) {
    return false;
}
#endif
