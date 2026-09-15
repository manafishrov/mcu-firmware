#ifndef BMI270_SENSOR_H
#define BMI270_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BMI270_SENSOR_OK,
    BMI270_SENSOR_NOT_INITIALIZED,
    BMI270_SENSOR_ARGUMENT_ERROR,
    BMI270_SENSOR_BUS_ERROR,
    BMI270_SENSOR_DEVICE_ERROR,
    BMI270_SENSOR_CONFIG_ERROR,
    BMI270_SENSOR_NOT_READY,
    BMI270_SENSOR_STALE,
    BMI270_SENSOR_TEMPERATURE_INVALID
} bmi270_sensor_result_t;

typedef struct {
    bool initialized;
    bmi270_sensor_result_t result;
    int8_t bosch_result;
    uint8_t chip_id;
    uint8_t error;
    uint8_t status;
    uint8_t internal_status;
    uint8_t config[4];    /* ACC_CONF, ACC_RANGE, GYR_CONF, GYR_RANGE (0x40..0x43). */
    uint8_t power[2];     /* PWR_CONF, PWR_CTRL (0x7c..0x7d). */
    uint32_t sensor_time; /* 24-bit counter, 39.0625 us/tick. */
    uint32_t samples;
    uint32_t bus_timeouts;
} bmi270_sensor_diagnostics_t;

/* Core 1 only: owns SPI1, GP10 SCK/11 MOSI/12 MISO/13 CS. Not thread-safe.
 * Performs Bosch reset/configuration upload and gyro startup wait; call while
 * outputs are inhibited, never in the 500 Hz loop. Explicit init is required
 * again after a bus, identity, device-status, or register-configuration error.
 * Success means configured, not that a fresh sample has been delivered.
 */
bool bmi270_sensor_init(void);

/* Non-waiting sample acquisition: false on missing data-ready, repeated sensor
 * time, invalid temperature, or any device/bus failure. Outputs are untouched
 * on false; callers must not count/reuse them as fresh or healthy. SPI work has
 * a 1 ms deadline (500 us per transaction), excluding interrupt preemption.
 * Returns raw, uncompensated acceleration (m/s^2), gyro (rad/s), temperature
 * (degrees C), with [+1,-1,-1] sensor signs and bmi270==0.4.3 conversion factors.
 * All three pointers are required. Do not infer health from initialized alone.
 */
bool bmi270_sensor_read(float accel[3], float gyro[3], float *temperature);

/* Same owning core only; copy into the coordinator's cross-core snapshot.
 * No I/O, logging, USB calls, allocation, or locking. NULL is allowed.
 */
void bmi270_sensor_get_diagnostics(bmi270_sensor_diagnostics_t *snapshot);

#endif
