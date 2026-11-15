#ifndef AM32_BOOTLOADER_MOCK_GPIO_H
#define AM32_BOOTLOADER_MOCK_GPIO_H
#include <pico/types.h>

#define GPIO_IN false
#define GPIO_OUT true

void gpio_init(uint pin);
void gpio_set_dir(uint pin, bool out);
void gpio_put(uint pin, bool value);
bool gpio_get(uint pin);
void gpio_pull_up(uint pin);
void gpio_disable_pulls(uint pin);
#endif
