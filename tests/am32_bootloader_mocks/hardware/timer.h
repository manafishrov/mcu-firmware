#ifndef AM32_BOOTLOADER_MOCK_TIMER_H
#define AM32_BOOTLOADER_MOCK_TIMER_H
#include <stdint.h>

uint32_t time_us_32(void);
void busy_wait_us_32(uint32_t delay_us);
#endif
