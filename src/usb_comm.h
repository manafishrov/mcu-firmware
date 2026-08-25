#ifndef USB_COMM_H
#define USB_COMM_H

#include <pico/time.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define USB_INPUT_START_BYTE 0x5A
#define USB_CONFIG_START_BYTE 0xC5
#define USB_INPUT_PACKET_SIZE(num_motors) (1 + ((num_motors) * 2) + 1)
#define USB_COMM_TIMEOUT_MS 200
#define USB_PACKET_TIMEOUT_MS 250

typedef enum {
    USB_PACKET_NONE = 0,
    USB_PACKET_COMMAND,
    USB_PACKET_CONFIG,
    USB_PACKET_ESC_FIRMWARE_CONTROL,
    USB_PACKET_ESC_FIRMWARE_DATA,
} usb_packet_kind_t;

typedef struct {
    uint8_t start_byte;
    uint8_t *buffer;
    size_t packet_size;
    size_t index;
    usb_packet_kind_t kind;
    absolute_time_t last_byte_time;
} usb_packet_reader_t;

uint8_t usb_calculate_checksum(const uint8_t *data, size_t len);
usb_packet_kind_t usb_poll(usb_packet_reader_t *readers, size_t reader_count);
usb_packet_kind_t usb_process_byte(usb_packet_reader_t *readers, size_t reader_count, uint8_t byte,
                                   absolute_time_t now);
void usb_expire_incomplete_packets(usb_packet_reader_t *readers, size_t reader_count,
                                   absolute_time_t now);
bool usb_parse_packet(const uint8_t *usb_buf, size_t packet_size, uint16_t *raw_values,
                      int num_motors, absolute_time_t *last_comm_time);
void usb_check_timeout(absolute_time_t last_comm_time, uint16_t *thruster_values, int num_motors,
                       uint16_t neutral_value, bool *comm_timed_out);

#endif
