#include "bmi270_sensor.h"

#include "bmi2.h"
#include "bmi270.h"
#include "bmi2_defs.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* The standalone host test supplies a recording SPI/timer implementation. */
#ifndef BMI270_SENSOR_TEST
#include "hardware/gpio.h"
#include "hardware/regs/spi.h"
#include "hardware/spi.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/timer.h"
#endif

#define BMI270_SPI_BAUD 4000000u
#define BMI270_PIN_SCK 10u
#define BMI270_PIN_MOSI 11u
#define BMI270_PIN_MISO 12u
#define BMI270_PIN_CS 13u
#define BMI270_TRANSFER_BYTES 64u
#define BMI270_TRANSACTION_US 500u
#define BMI270_READ_US 1000u
#define BMI270_INIT_US 1000000u
#define BMI270_TIME_MASK 0x00ffffffu
#define BMI270_TIME_HALF_RANGE 0x00800000u

static struct bmi2_dev device;
static bmi270_sensor_diagnostics_t diagnostics = {.result = BMI270_SENSOR_NOT_INITIALIZED};
static uint64_t operation_deadline;
static bool bus_available;

static bool fail(bmi270_sensor_result_t result, bool invalidate) {
    diagnostics.result = result;
    if (invalidate) {
        diagnostics.initialized = false;
    }
    return false;
}

static bool transport_timeout(void) {
    /* No blocking reset/drain on an error. Stop clocks and leave the device
     * deselected; a later explicit init resets the peripheral and its FIFOs.
     */
    spi_get_hw(spi1)->cr1 &= ~SPI_SSPCR1_SSE_BITS;
    gpio_put(BMI270_PIN_CS, true);
    if (bus_available) {
        diagnostics.bus_timeouts++;
    }
    bus_available = false;
    return fail(BMI270_SENSOR_BUS_ERROR, true);
}

static bool exchange_byte(uint8_t output, uint8_t *input, uint64_t deadline) {
    while (!spi_is_writable(spi1)) {
        if (time_us_64() >= deadline) {
            return transport_timeout();
        }
    }
    if (time_us_64() >= deadline) {
        return transport_timeout();
    }
    spi_get_hw(spi1)->dr = output;
    while (!spi_is_readable(spi1)) {
        if (time_us_64() >= deadline) {
            return transport_timeout();
        }
    }
    *input = (uint8_t)spi_get_hw(spi1)->dr;
    return true;
}

static BMI2_INTF_RETURN_TYPE transfer(uint8_t address, uint8_t *read_data,
                                      const uint8_t *write_data, uint32_t length) {
    const bool reading = read_data != NULL;
    if (!bus_available || length == 0 || length > BMI270_TRANSFER_BYTES + (reading ? 1u : 0u) ||
        (!reading && write_data == NULL)) {
        return BMI2_E_COM_FAIL;
    }

    uint64_t deadline = time_us_64() + BMI270_TRANSACTION_US;
    if (operation_deadline < deadline) {
        deadline = operation_deadline;
    }
    if (time_us_64() >= deadline) {
        (void)transport_timeout();
        return BMI2_E_COM_FAIL;
    }

    uint8_t ignored;
    gpio_put(BMI270_PIN_CS, false);
    busy_wait_us_32(1); /* Exceeds CS setup time, even at the fastest Pico clock. */
    bool success = exchange_byte(address, &ignored, deadline);
    for (uint32_t i = 0; success && i < length; ++i) {
        uint8_t received;
        success = exchange_byte(reading ? 0 : write_data[i], &received, deadline);
        if (success && reading) {
            read_data[i] = received;
        }
    }
    while (success && spi_is_busy(spi1)) {
        if (time_us_64() >= deadline) {
            success = transport_timeout();
        }
    }
    if (success && time_us_64() >= deadline) {
        success = transport_timeout();
    }
    busy_wait_us_32(1); /* CS hold time after the final SCK edge. */
    gpio_put(BMI270_PIN_CS, true);
    return success ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE spi_read(uint8_t address, uint8_t *data, uint32_t length,
                                      void *context) {
    (void)context;
    /* Bosch adds the read bit and includes ONE dummy byte in length. Return
     * that dummy in data[0]; bmi2_get_regs removes it. CS spans address, dummy,
     * and payload. Removing another dummy here shifts every register read.
     */
    return transfer(address, data, NULL, length);
}

static BMI2_INTF_RETURN_TYPE spi_write(uint8_t address, const uint8_t *data, uint32_t length,
                                       void *context) {
    (void)context;
    return transfer(address, NULL, data, length);
}

static void delay_us(uint32_t period, void *context) {
    (void)context;
    const uint64_t now = time_us_64();
    if (!bus_available) {
        return;
    }
    if (now >= operation_deadline || period > operation_deadline - now) {
        (void)transport_timeout();
        return;
    }
    busy_wait_us_32(period);
}

static bool bosch_ok(int8_t result) {
    diagnostics.bosch_result = result;
    if (result != BMI2_OK || !bus_available) {
        return fail(result == BMI2_E_COM_FAIL || !bus_available ? BMI270_SENSOR_BUS_ERROR
                                                                : BMI270_SENSOR_CONFIG_ERROR,
                    true);
    }
    return true;
}

static bool read_registers(uint8_t address, uint8_t *data, uint16_t length) {
    return bosch_ok(bmi2_get_regs(address, data, length, &device));
}

static bool check_identity(void) {
    uint8_t identity[4]; /* CHIP_ID, reserved, ERR_REG, STATUS. */
    if (!read_registers(BMI2_CHIP_ID_ADDR, identity, sizeof(identity))) {
        return false;
    }
    diagnostics.chip_id = identity[0];
    diagnostics.error = identity[2];
    diagnostics.status = identity[3];
    if (diagnostics.chip_id != BMI270_CHIP_ID || diagnostics.error != 0) {
        return fail(BMI270_SENSOR_DEVICE_ERROR, true);
    }
    return true;
}

static bool check_configuration(void) {
    if (!read_registers(BMI2_ACC_CONF_ADDR, diagnostics.config, sizeof(diagnostics.config)) ||
        !read_registers(BMI2_PWR_CONF_ADDR, diagnostics.power, sizeof(diagnostics.power))) {
        return false;
    }
    /* 800 Hz, normal bandwidth, performance filters + gyro noise performance;
     * +/-2 g and +/-1000 deg/s, OIS default +/-250, APS/FIFO wakeup disabled,
     * accelerometer, gyroscope and temperature enabled, auxiliary disabled.
     */
    const uint8_t expected_config[4] = {0xab, 0x00, 0xeb, 0x01};
    const uint8_t expected_power[2] = {0x00, 0x0e};
    if (memcmp(diagnostics.config, expected_config, sizeof(expected_config)) != 0 ||
        memcmp(diagnostics.power, expected_power, sizeof(expected_power)) != 0) {
        return fail(BMI270_SENSOR_CONFIG_ERROR, true);
    }
    return true;
}

static uint32_t sensor_time(const uint8_t raw[15]) {
    return (uint32_t)raw[12] | ((uint32_t)raw[13] << 8u) | ((uint32_t)raw[14] << 16u);
}

static int32_t signed_word(const uint8_t *data) {
    const uint32_t value = (uint32_t)data[0] | ((uint32_t)data[1] << 8u);
    /* Avoid implementation-defined unsigned-to-signed narrowing. */
    return value < 0x8000u ? (int32_t)value : (int32_t)value - 0x10000;
}

bool bmi270_sensor_init(void) {
    memset(&device, 0, sizeof(device));
    memset(&diagnostics, 0, sizeof(diagnostics));
    diagnostics.result = BMI270_SENSOR_NOT_INITIALIZED;
    bus_available = true;
    operation_deadline = time_us_64() + BMI270_INIT_US;

    gpio_init(BMI270_PIN_CS);
    gpio_put(BMI270_PIN_CS, true); /* Set the latch before enabling output. */
    gpio_set_dir(BMI270_PIN_CS, GPIO_OUT);
    (void)spi_init(spi1, BMI270_SPI_BAUD);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(BMI270_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(BMI270_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(BMI270_PIN_MISO, GPIO_FUNC_SPI);
    gpio_pull_down(BMI270_PIN_MISO); /* An open MISO must not look like a device. */
    delay_us(10000, NULL);           /* Power-on startup margin. */

    device.intf = BMI2_SPI_INTF;
    device.read = spi_read;
    device.write = spi_write;
    device.delay_us = delay_us;
    device.read_write_len = BMI270_TRANSFER_BYTES;
    /* A NULL config_file_ptr selects the official 8192-byte bmi270_config_file.
     * Bosch performs interface dummy reads both before ID and after soft reset.
     */
    const int8_t result = bmi270_init(&device);
    diagnostics.chip_id = device.chip_id;
    diagnostics.internal_status = device.load_status;
    if (!bosch_ok(result) || !bosch_ok(bmi2_set_adv_power_save(BMI2_DISABLE, &device))) {
        return false;
    }
    const uint8_t power_conf = 0;
    if (!bosch_ok(bmi2_set_regs(BMI2_PWR_CONF_ADDR, &power_conf, 1, &device))) {
        return false;
    }

    struct bmi2_sens_config config[2] = {{.type = BMI2_ACCEL,
                                          .cfg.acc = {.odr = BMI2_ACC_ODR_800HZ,
                                                      .bwp = BMI2_ACC_NORMAL_AVG4,
                                                      .filter_perf = BMI2_PERF_OPT_MODE,
                                                      .range = BMI2_ACC_RANGE_2G}},
                                         {.type = BMI2_GYRO,
                                          .cfg.gyr = {.odr = BMI2_GYR_ODR_800HZ,
                                                      .bwp = BMI2_GYR_NORMAL_MODE,
                                                      .filter_perf = BMI2_PERF_OPT_MODE,
                                                      .ois_range = BMI2_GYR_OIS_250,
                                                      .range = BMI2_GYR_RANGE_1000,
                                                      .noise_perf = BMI2_PERF_OPT_MODE}}};
    const uint8_t sensors[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_TEMP};
    if (!bosch_ok(bmi270_set_sensor_config(config, 2, &device)) ||
        !bosch_ok(bmi270_sensor_enable(sensors, 3, &device)) ||
        !bosch_ok(bmi2_set_adv_power_save(BMI2_DISABLE, &device))) {
        return false;
    }
    delay_us(80000, NULL); /* Gyro suspend-to-normal startup, outside control loop. */
    if (!check_identity() || !check_configuration() ||
        !read_registers(BMI2_INTERNAL_STATUS_ADDR, &diagnostics.internal_status, 1)) {
        return false;
    }
    if (diagnostics.internal_status != BMI2_INIT_OK) {
        return fail(BMI270_SENSOR_DEVICE_ERROR, true);
    }

    /* Drain any startup sample and establish a time baseline. Even the first
     * public read must prove that the sensor clock advanced since initialization.
     */
    uint8_t raw[15];
    if (!read_registers(BMI2_ACC_X_LSB_ADDR, raw, sizeof(raw))) {
        return false;
    }
    diagnostics.sensor_time = sensor_time(raw);
    diagnostics.initialized = true;
    diagnostics.result = BMI270_SENSOR_OK;
    return true;
}

bool bmi270_sensor_read(float accel[3], float gyro[3], float *temperature) {
    if (accel == NULL || gyro == NULL || temperature == NULL) {
        return fail(BMI270_SENSOR_ARGUMENT_ERROR, false);
    }
    if (!diagnostics.initialized) {
        return false; /* Preserve the reason initialization/previous I/O failed. */
    }
    operation_deadline = time_us_64() + BMI270_READ_US;
    if (!check_identity()) {
        return false;
    }
    const uint8_t ready = BMI2_DRDY_ACC | BMI2_DRDY_GYR;
    if ((diagnostics.status & ready) != ready) {
        return fail(BMI270_SENSOR_NOT_READY, false);
    }
    uint8_t raw[15];
    uint8_t thermal[3]; /* INTERNAL_STATUS followed by signed temperature. */
    if (!check_configuration() || !read_registers(BMI2_ACC_X_LSB_ADDR, raw, sizeof(raw)) ||
        !read_registers(BMI2_INTERNAL_STATUS_ADDR, thermal, sizeof(thermal))) {
        return false;
    }
    diagnostics.internal_status = thermal[0];
    if (diagnostics.internal_status != BMI2_INIT_OK) {
        return fail(BMI270_SENSOR_DEVICE_ERROR, true);
    }
    /* SPI has no ACK. Also reject an all-ones data/time burst if a line fault
     * affected that transaction but not the surrounding identity/status reads.
     */
    bool all_ones = true;
    for (size_t i = 0; i < sizeof(raw); ++i) {
        all_ones = all_ones && raw[i] == UINT8_MAX;
    }
    if (all_ones) {
        return fail(BMI270_SENSOR_BUS_ERROR, true);
    }
    const uint32_t timestamp = sensor_time(raw);
    const uint32_t delta = (timestamp - diagnostics.sensor_time) & BMI270_TIME_MASK;
    if (delta == 0 || delta >= BMI270_TIME_HALF_RANGE) {
        return fail(BMI270_SENSOR_STALE, false);
    }
    const int32_t raw_temperature = signed_word(&thermal[1]);
    if (raw_temperature == -32768) { /* Bosch's invalid/unavailable sentinel. */
        return fail(BMI270_SENSOR_TEMPERATURE_INVALID, false);
    }

    /* Deliberately do NOT call bmi2_get_sensor_data: it applies gyro ZX cross
     * sensitivity compensation. The Python baseline reads these raw registers.
     * Its bmi270==0.4.3 GRAVITY is 9.81288, not standard gravity 9.80665, and its
     * temperature factor is 0.001952594 (not exactly 1/512). Preserve both.
     */
    const float accel_scale = (2.0f * 9.81288f) / 32768.0f;
    const float gyro_scale = (1000.0f * 0.017453292519943295f) / 32768.0f;
    for (size_t axis = 0; axis < 3; ++axis) {
        const float sign = axis == 0 ? 1.0f : -1.0f;
        accel[axis] = (float)signed_word(&raw[axis * 2]) * accel_scale * sign;
        gyro[axis] = (float)signed_word(&raw[6 + (axis * 2)]) * gyro_scale * sign;
    }
    *temperature = (float)raw_temperature * 0.001952594f + 23.0f;
    diagnostics.sensor_time = timestamp;
    diagnostics.samples++;
    diagnostics.result = BMI270_SENSOR_OK;
    return true;
}

void bmi270_sensor_get_diagnostics(bmi270_sensor_diagnostics_t *snapshot) {
    if (snapshot != NULL) {
        *snapshot = diagnostics;
    }
}
