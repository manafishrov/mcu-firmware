#ifndef USB_RX_H
#define USB_RX_H
#include <pico/time.h>
#include <stdbool.h>

/* USB worker IRQ stamps ingress chunks; main-loop dequeue never renews a lease. */
void usb_rx_init(void);
int usb_rx_get(absolute_time_t *arrival);
bool usb_rx_take_overflow(void);
#endif
