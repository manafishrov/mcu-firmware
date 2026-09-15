#ifndef USB_BUFFER_MOCK_STDIO_H
#define USB_BUFFER_MOCK_STDIO_H

void stdio_set_chars_available_callback(void (*callback)(void *), void *context);
#endif
