# Current above idle

This is a reporting-only estimate for the Manafish ROV's two four-in-one
ESC boards. Channels 0–3 share one sensor and channels 4–7 share another.
Individual-sensor layouts are not supported by this estimator. Auto-zero does
not make the sensor topology irrelevant.

The AM60 raw reading decreases under load. Each board reports
`max(0, idle_baseline - mean_of_fresh_raw_readings)`. Duplicate readings from
its four controllers are averaged, not summed. The two corrected board values
are summed by the Pi. Raw readings are still forwarded unchanged for diagnosis.
The AM32 fixed 1820 mV / negative-slope correction must not also be installed.

## Acquiring and retaining a baseline

- All eight commands must be neutral and all eight reported eRPM values must
  be fresh and zero continuously for three seconds. Zero reported eRPM is not
  independent proof of physical standstill.
- Each board then needs a one-second window with at least ten fresh observations
  from every controller. Successive board samples cannot reuse observations;
  the board-mean span must stay within 2 A. Current and eRPM freshness is 500 ms.
- Baselines are independent per board, remain fixed during commanded or reported
  movement, and may be updated after another stable stopped interval.
- After calibration, remaining fresh duplicates can provide the board mean.
  Losing all current reports for a board invalidates its baseline. Returning
  samples cannot silently revive it; another stopped calibration is required.
- Startup, DShot reinitialization, protocol teardown, and disabled service clear
  calibration. PWM, ESC upload staging, and recovery cannot calibrate or provide
  valid current. A current-data gap is board-local; lost zero-eRPM continuity
  restarts the shared stopped interval.

## USB telemetry

Existing type 3 (`CURRENT`) remains the raw unsigned whole-amp EDT value for
all eight channels. It is a diagnostic sensor reading, not corrected current.
Two new packet types use the existing signed int32 payload and framing:

| Type | Meaning | Channel IDs | Units |
| --- | --- | --- | --- |
| 9 | Current above idle, per board | 0 and 4 | mA; -1 unavailable |
| 10 | Idle baseline, per board | 0 and 4 | mA; -1 unavailable |

Both are emitted every 100 ms, except during ESC upload staging or recovery. Milliamps preserve
averages; they do not add resolution to the original whole-amp EDT measurements.
The Pi must require both fresh corrected board reports and must not fall back
to raw current. Missing calibration or telemetry is not measured zero.

## Limits and installation order

Auto-zero removes an offset; it does not validate the ADC input, sensor gain,
voltage dependence, or response under load. Compare against an independent
current measurement before treating the result as amperes. It excludes the
idle load and is not absolute battery current, charge accounting, or protection.
AM32's measured-current limiter and consumed-mAh calculation still use its
uncorrected reading. Sensorless protection and EEPROM settings are unchanged;
read back the limiter setting before relying on its behavior.

Release and install a nullable-current-capable app before the matching Pi
firmware. Install AM32 with both the raw-current restoration and the existing
product input policy on all eight controllers using the old Pico firmware
first. Only then install a Pi bundle containing the new Pico firmware. Pi
auto-flashing means a simultaneous bundle upgrade does not enforce ESC-first
ordering. These source changes do not change release versions or pinned images.
Old Pi firmware ignores types 9/10 and still reports raw current; old apps cannot
consume the new nullable/fractional status contract.
