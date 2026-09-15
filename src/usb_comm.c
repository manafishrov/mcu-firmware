#include "usb_comm.h"
#include "log.h"
#include "usb_rx.h"
#include <pico/error.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

uint8_t usb_calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

void usb_expire_incomplete_packets(usb_packet_reader_t *readers, size_t reader_count,
                                   absolute_time_t now) {
    for (size_t i = 0; i < reader_count; ++i) {
        if (readers[i].index > 0 &&
            absolute_time_diff_us(readers[i].last_byte_time, now) >=
                (readers[i].kind == USB_PACKET_CONTROL ? 100 : USB_PACKET_TIMEOUT_MS) * 1000) {
            readers[i].index = 0;
        }
    }
}

usb_packet_kind_t usb_process_byte(usb_packet_reader_t *readers, size_t reader_count, uint8_t byte,
                                   absolute_time_t now) {
    usb_expire_incomplete_packets(readers, reader_count, now);
    usb_packet_reader_t *active_reader = NULL;

    for (size_t i = 0; i < reader_count; ++i) {
        if (readers[i].index > 0) {
            active_reader = &readers[i];
            break;
        }
    }

    if (active_reader != NULL) {
        active_reader->buffer[active_reader->index++] = byte;
        active_reader->last_byte_time = now;
        if (active_reader->kind == USB_PACKET_CONTROL && active_reader->index == 6) {
            size_t payload =
                (size_t)active_reader->buffer[4] | ((size_t)active_reader->buffer[5] << 8);
            if (payload > USB_CONTROL_MAX_PACKET_SIZE - 18 || active_reader->buffer[1] != 1 ||
                active_reader->buffer[3] != 0) {
                active_reader->index = 0;
                return USB_PACKET_INVALID;
            }
            active_reader->packet_size = payload + 18;
        }
        if (active_reader->index >= active_reader->packet_size) {
            usb_packet_kind_t kind = active_reader->kind;
            active_reader->index = 0;
            return kind;
        }
    } else {
        for (size_t i = 0; i < reader_count; ++i) {
            if (byte == readers[i].start_byte) {
                if (readers[i].kind == USB_PACKET_CONTROL) {
                    readers[i].packet_size = USB_CONTROL_MAX_PACKET_SIZE;
                }
                readers[i].buffer[0] = byte;
                readers[i].index = 1;
                readers[i].last_byte_time = now;
                break;
            }
        }
    }
    return USB_PACKET_NONE;
}

usb_packet_kind_t usb_poll(usb_packet_reader_t *readers, size_t reader_count) {
    absolute_time_t now = get_absolute_time();
    usb_expire_incomplete_packets(readers, reader_count, now);
    if (usb_rx_take_overflow()) {
        for (size_t i = 0; i < reader_count; ++i) {
            readers[i].index = 0;
        }
        return USB_PACKET_INVALID;
    }
    absolute_time_t arrival;
    int c = usb_rx_get(&arrival);
    size_t budget = 256;
    while (c != PICO_ERROR_TIMEOUT && budget-- > 0) {
        usb_packet_kind_t kind = usb_process_byte(readers, reader_count, (uint8_t)c, arrival);
        if (kind != USB_PACKET_NONE) {
            return kind;
        }
        if (budget == 0) {
            return USB_PACKET_NONE;
        }
        c = usb_rx_get(&arrival);
    }
    return USB_PACKET_NONE;
}

bool usb_parse_packet(const uint8_t *usb_buf, size_t packet_size, uint16_t *raw_values,
                      int num_motors, absolute_time_t *last_comm_time) {
    if (usb_buf[0] != USB_INPUT_START_BYTE) {
        return false;
    }

    uint8_t received_checksum = usb_buf[packet_size - 1];
    uint8_t calculated_checksum = usb_calculate_checksum(&usb_buf[0], packet_size - 1);
    if (received_checksum != calculated_checksum) {
        return false;
    }

    for (int i = 0; i < num_motors; ++i) {
        raw_values[i] = ((uint16_t)usb_buf[(2 * i) + 2] << 8) | usb_buf[(2 * i) + 1];
    }
    *last_comm_time = get_absolute_time();
    return true;
}

void usb_check_timeout(absolute_time_t last_comm_time, uint16_t *thruster_values, int num_motors,
                       uint16_t neutral_value, bool *comm_timed_out) {
    if (absolute_time_diff_us(last_comm_time, get_absolute_time()) > USB_COMM_TIMEOUT_MS * 1000) {
        for (int i = 0; i < num_motors; ++i) {
            thruster_values[i] = neutral_value;
        }
        if (!*comm_timed_out) {
            *comm_timed_out = true;
            log_warn("USB comm lost, motors neutral");
        }
    }
}
