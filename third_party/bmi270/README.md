# Bosch BMI270 SensorAPI

Vendored from [boschsensortec/BMI270_SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI/tree/41129fcfe39c583ee5462d79195741945d51c1fe),
commit `41129fcfe39c583ee5462d79195741945d51c1fe`.

The six upstream files below are unmodified, including their original line
endings and copyright notices. `bmi270.c` contains the official 8192-byte
`bmi270_config_file` image; no separate binary or generated image is needed.
`LICENSE` is Bosch's BSD-3-Clause license. Include it with binary distributions
as required by its second clause. The workspace's license does not replace it.

| File | SHA-256 |
| --- | --- |
| `bmi2.c` | `8d42ba3281b8448f674a4df616a7e176a259ad3c38c53b6d2c49bfdbdb0a82db` |
| `bmi2.h` | `a86c53c480974a0c56e24d9d4e736445c61ebc07addec6d3eaceddb8a2c97cd3` |
| `bmi2_defs.h` | `39c2cf19bfbf735cb0ed209efd975a731be50dff71b432b24a6a2625b2b14304` |
| `bmi270.c` | `f6446867378bbcc90ea97ef6593555de8767ca20b799355c1b37c2cabb7b7b41` |
| `bmi270.h` | `7979ec4c943e2825351bd9d3d267b63abc6f8314c1f13f1c37e95cabc3fc82c8` |
| `LICENSE` | `56df116398e0b72a12ae353d93002d29175583590283ced2c2510e3d8075546f` |

## Integration

Compile `bmi2.c`, `bmi270.c`, and `../../src/imu/bmi270_sensor.c`.
Add this directory as a system include path and link Pico SDK `hardware_spi`
and `pico_stdlib`. Keep the upstream files out of project formatting and lint
rewrites. They do not need any Bosch example `common.c`, I2C driver, or malloc.

The wrapper uses the Bosch API for reset, image upload, sensor configuration,
and register access. Acquisition reads raw registers instead of
`bmi2_get_sensor_data`, which would add gyro cross-axis compensation absent
from the Python baseline. Bosch still reads its correction coefficient during
initialization; the wrapper never applies it.

SPI1 uses mode 0, 8-bit MSB-first frames, 4 MHz, GP10 SCK, GP11 MOSI, GP12 MISO,
and software-controlled GP13 CS. One CS assertion spans address, read dummy
byte, and data. The Bosch API strips the dummy byte; the transport must not
strip it again. Uploads use 64-byte chunks. FIFO polling has a 500 us deadline
per transaction, within a 1 ms read budget or 1 s initialization budget. These
are software deadlines, not a bound on interrupt preemption. The wrapper does
not disable interrupts or claim exclusive use of the CPU.

Initialization enables acceleration, gyro, and temperature, and verifies:

| Registers | Expected bytes | Meaning |
| --- | --- | --- |
| `0x00` | `24` | BMI270 chip ID |
| `0x21` | `01` | Configuration initialization succeeded |
| `0x40..0x43` | `ab 00 eb 01` | 800 Hz accel/gyro, normal bandwidth, performance filtering/noise, +/-2 g, +/-1000 deg/s |
| `0x7c..0x7d` | `00 0e` | APS off, accel/gyro/temperature on, auxiliary off |

Read calls check identity, error/status, configuration, both data-ready bits,
and forward movement of the 24-bit sensor clock. An all-`ff` data/time burst is
rejected even when surrounding status reads look valid. A contiguous 15-byte
burst reads accel/gyro/time using the sensor's register shadowing. Temperature is a
separate register read; it updates more slowly than accel/gyro. Unchanged raw
values alone are not a fault: a stationary device can produce identical samples.
Freshness checks cannot detect a sensor that lies about both its data-ready and
clock while replaying plausible data.

A false read leaves all caller outputs unchanged. The caller must not process
or transmit them as a fresh sample. Bus, identity, error-status, and configuration
failures invalidate initialization and require an explicit reinitialization
while outputs are inhibited. No USB or logging calls run in the sensor wrapper.
The owning core may copy diagnostics into its normal cross-core snapshot.

Conversions retain `bmi270==0.4.3` compatibility, including its gravity constant
`9.81288 m/s^2` and temperature factor `0.001952594`, with `[+1,-1,-1]` signs.
Changing these to standard gravity or exactly `1/512` would change the baseline.

## Host regression

From the repository root:

```sh
cc -std=c11 -Wall -Wextra -Werror -DBMI270_SENSOR_TEST \
  -Ithird_party/bmi270 tests/test_bmi270_sensor.c \
  third_party/bmi270/bmi2.c third_party/bmi270/bmi270.c \
  -lm -o /tmp/test_bmi270_sensor
/tmp/test_bmi270_sensor
```

The test supplies a byte-level SPI peripheral and links the real Bosch API.
It checks the complete uploaded image, configuration, raw conversion over the
full signed 16-bit domain, dummy handling, freshness and wraparound, invalid
temperature, disconnection,
configuration drift, and TX/RX/busy and whole-read timeout paths. It performs
no hardware access. Physical identity, signal integrity, sample rates, and
execution times still require coordinator-owned bench measurements.
