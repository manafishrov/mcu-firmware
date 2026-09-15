#ifndef SPI_PROBE_MOCK_SDK_H
#define SPI_PROBE_MOCK_SDK_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef unsigned int uint;
typedef struct {
    volatile uint32_t dr, cr1;
} spi_hw_t;
typedef struct {
    struct {
        uint32_t status;
    } io[30];
} mock_io_bank_t;
extern spi_hw_t *const spi1;
extern mock_io_bank_t *const io_bank0_hw;
#define GPIO_OUT true
#define GPIO_IN false
#define GPIO_FUNC_SPI 1u
#define GPIO_FUNC_SIO 5u
#define GPIO_OVERRIDE_LOW 2u
#define SPI_CPOL_0 0u
#define SPI_CPOL_1 1u
#define SPI_CPHA_0 0u
#define SPI_CPHA_1 1u
#define SPI_MSB_FIRST 0u
#define SPI_SSPCR1_SSE_BITS 2u
#define IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS (1u << 17u)
#define SPI_PROBE_SOURCE_SHA256 "offline-test"

uint64_t time_us_64(void);
void busy_wait_us_32(uint32_t us);
void sleep_us(uint64_t us);
void sleep_ms(uint32_t ms);
void tight_loop_contents(void);
void __wfe(void);
void __sev(void);
void gpio_init(uint pin);
void gpio_put(uint pin, bool value);
void gpio_set_dir(uint pin, bool output);
void gpio_set_function(uint pin, uint function);
uint gpio_get_function(uint pin);
void gpio_set_outover(uint pin, uint value);
void gpio_pull_down(uint pin);
void gpio_pull_up(uint pin);
void gpio_disable_pulls(uint pin);
bool gpio_get_pad(uint pin);
bool gpio_get(uint pin);
uint32_t gpio_get_all(void);
uint spi_init(spi_hw_t *spi, uint baud);
void spi_set_format(spi_hw_t *spi, uint bits, uint polarity, uint phase, uint order);
uint spi_get_baudrate(spi_hw_t *spi);
spi_hw_t *spi_get_hw(spi_hw_t *spi);
bool spi_is_writable(spi_hw_t *spi);
bool spi_is_readable(spi_hw_t *spi);
bool spi_is_busy(spi_hw_t *spi);
int spi_write_read_blocking(spi_hw_t *spi, const uint8_t *tx, uint8_t *rx, size_t length);
void multicore_launch_core1(void (*entry)(void));
void watchdog_disable(void);
bool stdio_usb_init(void);
uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t state);
bool tud_cdc_connected(void);
uint32_t tud_cdc_write_available(void);
uint32_t tud_cdc_write(const void *data, uint32_t length);
uint32_t tud_cdc_write_flush(void);
#endif
