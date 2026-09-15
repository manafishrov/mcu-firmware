#ifndef USB_BUFFER_MOCK_TIME_H
#define USB_BUFFER_MOCK_TIME_H
#include <stdint.h>

typedef struct {
    uint64_t us_since_boot;
} absolute_time_t;
absolute_time_t get_absolute_time(void);
#endif
