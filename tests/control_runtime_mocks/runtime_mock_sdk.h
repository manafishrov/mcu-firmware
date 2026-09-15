#ifndef CONTROL_RUNTIME_MOCK_SDK_H
#define CONTROL_RUNTIME_MOCK_SDK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;
typedef uint64_t absolute_time_t;
struct repeating_timer;
typedef bool (*repeating_timer_callback_t)(struct repeating_timer *);
struct repeating_timer {
    repeating_timer_callback_t callback;
};

#define PICO_ERROR_TIMEOUT (-1)
#define GPIO_OVERRIDE_NORMAL 0u
#define GPIO_OVERRIDE_LOW 2u

uint32_t time_us_32(void);
uint64_t time_us_64(void);
absolute_time_t get_absolute_time(void);
absolute_time_t from_us_since_boot(uint64_t us);
int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to);
void busy_wait_until(absolute_time_t deadline);
bool add_repeating_timer_us(int64_t delay, repeating_timer_callback_t callback, void *user_data,
                            struct repeating_timer *timer);
void multicore_launch_core1_with_stack(void (*entry)(void), uint32_t *stack, size_t size);
void watchdog_enable(uint32_t delay_ms, bool pause_on_debug);
void watchdog_update(void);
void tight_loop_contents(void);
void gpio_set_outover(uint pin, uint value);
void pwm_set_gpio_level(uint pin, uint16_t value);
uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t state);
void __dmb(void);
void stdio_set_chars_available_callback(void (*callback)(void *), void *context);
uint32_t tud_cdc_available(void);
uint32_t tud_cdc_read(void *buffer, uint32_t size);
bool tud_cdc_connected(void);
uint32_t tud_cdc_write_available(void);
uint32_t tud_cdc_write(const void *buffer, uint32_t size);
uint32_t tud_cdc_write_flush(void);

#endif
