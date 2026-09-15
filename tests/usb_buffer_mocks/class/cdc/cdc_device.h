#ifndef USB_BUFFER_MOCK_CDC_DEVICE_H
#define USB_BUFFER_MOCK_CDC_DEVICE_H
#include <stdbool.h>
#include <stdint.h>

bool tud_cdc_connected(void);
uint32_t tud_cdc_write_available(void);
uint32_t tud_cdc_write(const void *buffer, uint32_t length);
uint32_t tud_cdc_write_flush(void);
uint32_t tud_cdc_available(void);
uint32_t tud_cdc_read(void *buffer, uint32_t length);
#endif
