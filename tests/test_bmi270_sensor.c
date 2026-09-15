/* Standalone SPI-level test, linking the REAL, unmodified Bosch driver.
 * cc -std=c11 -Wall -Wextra -Werror -DBMI270_SENSOR_TEST -Ithird_party/bmi270 \
 *   tests/test_bmi270_sensor.c third_party/bmi270/bmi2.c \
 *   third_party/bmi270/bmi270.c -lm -o /tmp/test_bmi270_sensor
 * /tmp/test_bmi270_sensor
 * No hardware or Pico SDK is used. Guarded to coexist with the Unity wildcard.
 */
#ifdef BMI270_SENSOR_TEST
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bmi270.h"

#define GPIO_OUT 1
#define GPIO_FUNC_SPI 2
#define SPI_CPOL_0 0
#define SPI_CPHA_0 0
#define SPI_MSB_FIRST 0
#define SPI_SSPCR1_SSE_BITS 2u

typedef struct {
    uint32_t dr;
    uint32_t cr1;
} spi_hw_t;
static spi_hw_t peripheral;
static spi_hw_t *const spi1 = &peripheral;
static uint8_t registers[128];
static uint8_t uploaded[8192];
static uint32_t uploaded_count;
static uint64_t clock_us;
static uint32_t byte_cost_us;
static bool cs_high;
static uint8_t address;
static uint32_t transaction_index;
static bool read_transaction;
static uint32_t transactions;
static bool stall_tx;
static bool stall_rx;
static bool stall_busy;
static bool unplugged;
static bool reject_image;
static bool reject_config;
static bool drained_data;
static bool miso_pulldown;
static uint32_t functions[30];

static void reset_registers(void) {
    memset(registers, 0, sizeof(registers));
    registers[0] = BMI270_CHIP_ID;
    registers[BMI2_PWR_CONF_ADDR] = 3;
    registers[BMI2_ACC_CONF_ADDR] = 0xa8;
    registers[BMI2_GYR_CONF_ADDR] = 0xa9;
    registers[BMI2_GYR_CONF_ADDR + 1] = 0;
    /* Nonzero compensation must not affect our raw gyro output. */
    registers[BMI2_FEATURES_REG_ADDR + BMI270_GYRO_CROSS_SENSE_STRT_ADDR] = 21;
    registers[BMI2_SENSORTIME_ADDR] = 0x40;
    registers[BMI2_SENSORTIME_ADDR + 1] = 0x01;
    uploaded_count = 0;
}

static uint64_t time_us_64(void) {
    return clock_us++;
}
static void busy_wait_us_32(uint32_t delay) {
    clock_us += delay;
}
static void gpio_init(uint32_t pin) {
    assert(pin == 13);
}
static void gpio_set_dir(uint32_t pin, int direction) {
    assert(pin == 13 && direction == GPIO_OUT && cs_high);
}
static void gpio_set_function(uint32_t pin, uint32_t function) {
    assert(pin >= 10 && pin <= 12 && function == GPIO_FUNC_SPI);
    functions[pin] = function;
}
static void gpio_pull_down(uint32_t pin) {
    assert(pin == 12);
    miso_pulldown = true;
}
static void gpio_put(uint32_t pin, bool high) {
    assert(pin == 13);
    if (!high) {
        assert(cs_high);
        transaction_index = 0;
        transactions++;
        drained_data = false;
    } else if (drained_data) {
        registers[BMI2_STATUS_ADDR] &= ~(BMI2_DRDY_ACC | BMI2_DRDY_GYR);
    }
    cs_high = high;
}
static uint32_t spi_init(spi_hw_t *spi, uint32_t baud) {
    assert(spi == spi1 && baud == 4000000);
    peripheral = (spi_hw_t){.cr1 = SPI_SSPCR1_SSE_BITS};
    return baud;
}
static void spi_set_format(spi_hw_t *spi, int bits, int polarity, int phase, int order) {
    assert(spi == spi1 && bits == 8 && polarity == 0 && phase == 0 && order == 0);
}
static spi_hw_t *spi_get_hw(spi_hw_t *spi) {
    assert(spi == spi1);
    return spi;
}
static bool spi_is_writable(spi_hw_t *spi) {
    assert(spi == spi1);
    return !stall_tx;
}
static bool spi_is_busy(spi_hw_t *spi) {
    assert(spi == spi1);
    return stall_busy;
}
static bool spi_is_readable(spi_hw_t *spi) {
    assert(spi == spi1 && !cs_high);
    if (stall_rx) {
        return false;
    }
    clock_us += byte_cost_us;
    const uint8_t output = (uint8_t)spi->dr;
    uint8_t input = 0xa5; /* Nonzero dummy catches off-by-one dummy handling. */
    if (transaction_index == 0) {
        address = output & 0x7f;
        read_transaction = (output & 0x80) != 0;
    } else if (read_transaction) {
        assert(output == 0);
        if (transaction_index >= 2) {
            const uint32_t reg = address + transaction_index - 2;
            assert(reg < sizeof(registers));
            input = unplugged ? 0xff : registers[reg];
            if (reg == BMI2_SENSORTIME_ADDR + 2 && address == BMI2_ACC_X_LSB_ADDR) {
                drained_data = true;
            }
        }
    } else if (!unplugged) {
        if (address == BMI2_INIT_DATA_ADDR) {
            const uint32_t word = (uint32_t)registers[BMI2_INIT_ADDR_0] |
                                  ((uint32_t)registers[BMI2_INIT_ADDR_1] << 4u);
            const uint32_t index = word * 2u + transaction_index - 1u;
            assert(index < sizeof(uploaded));
            uploaded[index] = output;
            uploaded_count++;
        } else {
            const uint32_t reg = address + transaction_index - 1;
            assert(reg < sizeof(registers));
            if (reg == BMI2_CMD_REG_ADDR && output == BMI2_SOFT_RESET_CMD) {
                reset_registers();
            } else if (!(reject_config && reg == BMI2_ACC_CONF_ADDR)) {
                registers[reg] = output;
            }
            if (reg == BMI2_INIT_CTRL_ADDR && output == 1) {
                assert(uploaded_count == sizeof(uploaded));
                registers[BMI2_INTERNAL_STATUS_ADDR] = reject_image ? BMI2_INIT_ERR : BMI2_INIT_OK;
            }
        }
    }
    transaction_index++;
    assert(transaction_index <= 66); /* Address plus <=64 data, or <=65 read bytes. */
    spi->dr = input;
    return true;
}

/* Include the real wrapper to test callbacks and timeout paths without making
 * private implementation details part of its public firmware API.
 */
#include "../src/imu/bmi270_sensor.c"

extern const uint8_t bmi270_config_file[];

static void reset_fixture(void) {
    peripheral = (spi_hw_t){0};
    clock_us = 0;
    byte_cost_us = 2;
    cs_high = true;
    transactions = 0;
    stall_tx = false;
    stall_rx = false;
    stall_busy = false;
    unplugged = false;
    reject_image = false;
    reject_config = false;
    drained_data = false;
    miso_pulldown = false;
    memset(functions, 0, sizeof(functions));
    reset_registers();
}

static void set_sample(uint32_t timestamp, const int16_t words[7]) {
    for (size_t i = 0; i < 6; ++i) {
        const uint16_t word = (uint16_t)words[i];
        registers[BMI2_ACC_X_LSB_ADDR + i * 2] = (uint8_t)word;
        registers[BMI2_ACC_X_LSB_ADDR + i * 2 + 1] = (uint8_t)(word >> 8u);
    }
    registers[BMI2_TEMPERATURE_0_ADDR] = (uint8_t)words[6];
    registers[BMI2_TEMPERATURE_1_ADDR] = (uint8_t)((uint16_t)words[6] >> 8u);
    for (size_t i = 0; i < 3; ++i) {
        registers[BMI2_SENSORTIME_ADDR + i] = (uint8_t)(timestamp >> (i * 8u));
    }
    registers[BMI2_STATUS_ADDR] = BMI2_DRDY_ACC | BMI2_DRDY_GYR;
}

static void check_failed_read(bmi270_sensor_result_t expected) {
    float accel[3] = {11, 12, 13};
    float gyro[3] = {14, 15, 16};
    float temperature = 17;
    const uint32_t previous_samples = diagnostics.samples;
    assert(!bmi270_sensor_read(accel, gyro, &temperature));
    assert(diagnostics.result == expected);
    assert(accel[0] == 11 && accel[1] == 12 && accel[2] == 13);
    assert(gyro[0] == 14 && gyro[1] == 15 && gyro[2] == 16 && temperature == 17);
    assert(diagnostics.samples == previous_samples);
    assert(cs_high);
}

static void test_initialization(void) {
    reset_fixture();
    assert(bmi270_sensor_init());
    assert(cs_high && miso_pulldown);
    for (size_t pin = 10; pin <= 12; ++pin) {
        assert(functions[pin] == GPIO_FUNC_SPI);
    }
    assert(uploaded_count == sizeof(uploaded));
    assert(memcmp(uploaded, bmi270_config_file, sizeof(uploaded)) == 0);
    assert(device.gyr_cross_sens_zx == 21);
    assert(diagnostics.initialized && diagnostics.samples == 0);
    assert(diagnostics.chip_id == 0x24 && diagnostics.internal_status == 1);
    assert(diagnostics.config[0] == 0xab && diagnostics.config[1] == 0);
    assert(diagnostics.config[2] == 0xeb && diagnostics.config[3] == 1);
    assert(diagnostics.power[0] == 0 && diagnostics.power[1] == 0x0e);
    assert(clock_us >= 110000 && clock_us < BMI270_INIT_US);
    check_failed_read(BMI270_SENSOR_NOT_READY);
}

static void test_raw_conversion(void) {
    reset_fixture();
    assert(bmi270_sensor_init());
    float accel[3], gyro[3], temperature;
    const int16_t words[7] = {-32768, 16384, 32767, 12345, -23456, 32767, -1024};
    set_sample(diagnostics.sensor_time, words);
    check_failed_read(BMI270_SENSOR_STALE); /* First read also needs clock progress. */
    set_sample(diagnostics.sensor_time + 51u, words);
    assert(bmi270_sensor_read(accel, gyro, &temperature));
    for (size_t axis = 0; axis < 3; ++axis) {
        const double sign = axis == 0 ? 1.0 : -1.0;
        const double a = sign * (double)words[axis] / 32768.0 * (2 * 9.81288);
        const double g = sign * 0.017453292519943295 * (double)words[3 + axis] / 32768.0 * 1000.0;
        assert(fabs((double)accel[axis] - a) < 0.000003);
        assert(fabs((double)gyro[axis] - g) < 0.000003);
    }
    assert(fabs((double)temperature - ((double)words[6] * 0.001952594 + 23.0)) < 0.000003);
    assert(diagnostics.samples == 1 && diagnostics.result == BMI270_SENSOR_OK);
    /* No ready: identical payload is not a fresh sample. */
    check_failed_read(BMI270_SENSOR_NOT_READY);
    /* Stuck asserted ready + repeated sensor time is not healthy either. */
    set_sample(diagnostics.sensor_time, words);
    check_failed_read(BMI270_SENSOR_STALE);
    /* An unchanged stationary payload with a NEW clock/DRDY is legitimate. */
    set_sample(diagnostics.sensor_time + 51u, words);
    assert(bmi270_sensor_read(accel, gyro, &temperature));
    /* Backwards clock (reset) is rejected. */
    set_sample(diagnostics.sensor_time - 10u, words);
    check_failed_read(BMI270_SENSOR_STALE);
    /* Natural 24-bit wrap is accepted. */
    diagnostics.sensor_time = 0xfffff0;
    set_sample(0x23, words);
    assert(bmi270_sensor_read(accel, gyro, &temperature));
    assert(diagnostics.sensor_time == 0x23);
    int16_t invalid[7];
    memcpy(invalid, words, sizeof(invalid));
    invalid[6] = -32768;
    set_sample(0x60, invalid);
    check_failed_read(BMI270_SENSOR_TEMPERATURE_INVALID);
    bmi270_sensor_diagnostics_t snapshot;
    bmi270_sensor_get_diagnostics(&snapshot);
    assert(snapshot.samples == diagnostics.samples && snapshot.sensor_time == 0x23);
    bmi270_sensor_get_diagnostics(NULL);
    assert(!bmi270_sensor_read(NULL, gyro, &temperature));
    assert(!bmi270_sensor_read(accel, NULL, &temperature));
    assert(!bmi270_sensor_read(accel, gyro, NULL));
    assert(diagnostics.result == BMI270_SENSOR_ARGUMENT_ERROR);
}

static void test_full_int16_domain(void) {
    reset_fixture();
    assert(bmi270_sensor_init());
    float accel[3], gyro[3], temperature;
    for (int32_t value = -32768; value <= 32767; ++value) {
        const int16_t raw = (int16_t)value;
        const int16_t thermal = value == -32768 ? 0 : raw;
        const int16_t words[7] = {raw, raw, raw, raw, raw, raw, thermal};
        set_sample(diagnostics.sensor_time + 51u, words);
        assert(bmi270_sensor_read(accel, gyro, &temperature));
        const double expected_accel = (double)value / 32768.0 * (2.0 * 9.81288);
        const double expected_gyro = 0.017453292519943295 * (double)value / 32768.0 * 1000.0;
        for (size_t axis = 0; axis < 3; ++axis) {
            const double sign = axis == 0 ? 1.0 : -1.0;
            assert(fabs((double)accel[axis] - sign * expected_accel) < 0.000003);
            assert(fabs((double)gyro[axis] - sign * expected_gyro) < 0.000003);
        }
        assert(fabs((double)temperature - ((double)thermal * 0.001952594 + 23.0)) < 0.00001);
    }
    assert(diagnostics.samples == 65536);
}

static void test_init_failures(void) {
    reset_fixture();
    registers[0] = 0;
    assert(!bmi270_sensor_init());
    assert(!diagnostics.initialized && diagnostics.chip_id == 0);
    reset_fixture();
    unplugged = true;
    assert(!bmi270_sensor_init());
    assert(!diagnostics.initialized && diagnostics.chip_id == 0xff);
    reset_fixture();
    reject_image = true;
    assert(!bmi270_sensor_init());
    assert(diagnostics.bosch_result == BMI2_E_CONFIG_LOAD);
    reset_fixture();
    reject_config = true;
    assert(!bmi270_sensor_init());
    assert(diagnostics.result == BMI270_SENSOR_CONFIG_ERROR);
    reset_fixture();
    stall_rx = true;
    assert(!bmi270_sensor_init());
    assert(cs_high && !diagnostics.initialized && diagnostics.bus_timeouts == 1);
    assert(clock_us < 12000);
}

static void test_runtime_faults(void) {
    const int16_t words[7] = {1, 2, 3, 4, 5, 6, 7};
    reset_fixture();
    assert(bmi270_sensor_init());
    set_sample(1000, words);
    registers[BMI2_STATUS_ADDR] = BMI2_DRDY_ACC; /* Both axes are mandatory. */
    check_failed_read(BMI270_SENSOR_NOT_READY);
    registers[BMI2_STATUS_ADDR] = BMI2_DRDY_GYR;
    check_failed_read(BMI270_SENSOR_NOT_READY);
    set_sample(1000, words);
    unplugged = true;
    check_failed_read(BMI270_SENSOR_DEVICE_ERROR);
    assert(!diagnostics.initialized);
    unplugged = false;
    const uint32_t previous_transactions = transactions;
    check_failed_read(BMI270_SENSOR_DEVICE_ERROR);
    assert(transactions == previous_transactions); /* Explicit init required. */
    assert(bmi270_sensor_init());
    set_sample(1000, words);
    registers[2] = 0x40;
    check_failed_read(BMI270_SENSOR_DEVICE_ERROR);
    assert(bmi270_sensor_init());
    set_sample(1000, words);
    registers[BMI2_INTERNAL_STATUS_ADDR] = BMI2_INIT_ERR;
    check_failed_read(BMI270_SENSOR_DEVICE_ERROR);
    assert(bmi270_sensor_init());
    set_sample(1000, words);
    registers[BMI2_ACC_CONF_ADDR + 1] = 1;
    check_failed_read(BMI270_SENSOR_CONFIG_ERROR);
    assert(bmi270_sensor_init());
    set_sample(1000, words);
    registers[BMI2_PWR_CONF_ADDR] = 1;
    check_failed_read(BMI270_SENSOR_CONFIG_ERROR);
    assert(bmi270_sensor_init());
    /* A single bad SPI data burst can occur between valid status reads. Pick
     * a baseline where an all-ones time would otherwise count as forward.
     */
    const int16_t all_ones[7] = {-1, -1, -1, -1, -1, -1, -1};
    diagnostics.sensor_time = 0xffffd0;
    set_sample(0xffffff, all_ones);
    check_failed_read(BMI270_SENSOR_BUS_ERROR);
    assert(!diagnostics.initialized);
}

static void test_transport_bounds(void) {
    for (size_t fault = 0; fault < 3; ++fault) {
        reset_fixture();
        assert(bmi270_sensor_init());
        stall_tx = fault == 0;
        stall_rx = fault == 1;
        stall_busy = fault == 2;
        const uint64_t start = clock_us;
        check_failed_read(BMI270_SENSOR_BUS_ERROR);
        assert(clock_us - start <= BMI270_TRANSACTION_US + 10u);
        assert(diagnostics.bus_timeouts == 1 && !diagnostics.initialized);
        assert((peripheral.cr1 & SPI_SSPCR1_SSE_BITS) == 0);
        stall_tx = stall_rx = stall_busy = false;
        assert(bmi270_sensor_init()); /* Peripheral recovery only on explicit init. */
    }
    reset_fixture();
    assert(bmi270_sensor_init());
    uint8_t bytes[66] = {0};
    const uint32_t before = transactions;
    assert(spi_read(0x80, bytes, sizeof(bytes), NULL) != BMI2_INTF_RET_SUCCESS);
    assert(spi_write(0, bytes, 65, NULL) != BMI2_INTF_RET_SUCCESS);
    assert(spi_write(0, NULL, 1, NULL) != BMI2_INTF_RET_SUCCESS);
    assert(transactions == before && cs_high);
    const int16_t words[7] = {1, 2, 3, 4, 5, 6, 7};
    set_sample(1000, words);
    byte_cost_us = 25; /* Each transaction fits 500 us, all reads exceed 1 ms. */
    const uint64_t start = clock_us;
    check_failed_read(BMI270_SENSOR_BUS_ERROR);
    assert(clock_us - start <= BMI270_READ_US + 30u);
    assert(clock_us - start >= BMI270_READ_US);
}

int main(void) {
    test_initialization();
    test_raw_conversion();
    test_full_int16_domain();
    test_init_failures();
    test_runtime_faults();
    test_transport_bounds();
    puts("BMI270: Bosch image upload, configuration, raw conversion, freshness and timeout tests "
         "passed");
    return 0;
}
#endif
