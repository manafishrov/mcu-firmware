#ifndef USB_TX_H
#define USB_TX_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Core 0 only. Whole packets are queued or dropped, never partially enqueued. */
bool usb_tx_packet(const uint8_t *data, size_t length, bool priority);
void usb_tx_service(void);
void usb_tx_discard_telemetry(void);
uint32_t usb_tx_dropped(void);
#endif
