#include "runtime.h"
#include "controller.h"
#include "imu/bmi270_sensor.h"
#include "log.h"
#include "motors.h"
#include "protocol.h"
#include "pwm/control.h"
#include "usb_tx.h"
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>
#include <math.h>
#include <pico/multicore.h>
#include <pico/platform.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef CONTROL_BUILD_IDENTITY
#define CONTROL_BUILD_IDENTITY "pico-control-dev:unidentified"
#endif

#define EVENT_COUNT 16u
#define SNAPSHOT_COUNT 8u
#define PERIOD_US 2000u
#define HOST_TIMEOUT_US 200000u
#define OUTPUT_TIMEOUT_US 10000u
#define PRESSURE_TIMEOUT_US 500000u
#define SETTINGS_TIMEOUT_US 5000000u
#define FRAME_ACK 0x80u
#define FRAME_CAPS 0x81u
#define FRAME_ATTITUDE 0x90u
#define FRAME_STATS 0x91u
#define FRAME_IMU 0x92u
#define MSG_HELLO 0x01u
#define MSG_CONTROL 0x10u
#define MSG_PRESSURE 0x11u
#define MSG_ATTITUDE 0x12u
#define MSG_DEPTH 0x13u
#define MSG_RAW 0x14u
#define MSG_BEGIN 0x20u
#define MSG_CHUNK 0x21u
#define MSG_COMMIT 0x22u
#define MSG_ABORT 0x23u
#define MSG_QUERY 0x24u
#define RESULT_APPLIED 0u
#define RESULT_STAGED 1u
#define RESULT_INVALID 2u
#define RESULT_BUSY 3u
#define RESULT_STALE 4u
#define RESULT_NOT_READY 5u
#define RESULT_UNSUPPORTED 6u

typedef struct {
    uint32_t type, session, sequence, received_us;
    uint32_t generation, digest;
    control_settings_t settings;
    float values[10];
    uint32_t flags;
} control_event_t;
typedef struct {
    uint64_t elapsed_us;
    uint32_t ahrs, pid, depth, missed, maximum, minimum, errors;
    uint32_t histogram[8];
    uint64_t execution_sum;
    uint32_t host_max, output_max, overflows;
} execution_stats_t;
typedef struct {
    control_output_t output;
    control_sample_t sample;
    bmi270_sensor_diagnostics_t sensor;
    float temperature;
    uint32_t session, sequence, command_us, completed_us, generation, digest;
    uint32_t health, stats_serial, committed_sequence;
    execution_stats_t stats;
} control_snapshot_t;
typedef struct {
    uint32_t session, sequence, type, generation, digest, result;
} application_ack_t;

static control_runtime_hooks_t callbacks;
static control_event_t events[EVENT_COUNT];
static volatile uint32_t event_read, event_write;
static control_snapshot_t snapshots[SNAPSHOT_COUNT];
static volatile uint32_t snapshot_read, snapshot_write;
static application_ack_t acknowledgements[EVENT_COUNT];
static volatile uint32_t ack_read, ack_write;
static volatile uint32_t shared_session, core0_heartbeat, core1_heartbeat;
static volatile uint32_t safety_armed, safety_latched, safety_host_us, safety_output_us;
static volatile uint32_t maintenance, physical_protocol, queue_overflows;
static volatile uint32_t accepted_floor;
static volatile uint32_t commit_authorized_sequence;
static volatile uint32_t sensor_retry_requested, sensor_retry_inhibited, output_permitted;
static struct repeating_timer safety_timer;
static uint32_t core1_stack[2048];
static uint32_t active_session, last_sequence, last_request_crc;
static uint32_t active_generation, active_digest, telemetry_sequence;
static uint32_t active_commit_sequence, safety_failed_session;
static uint32_t pending_sequence, pending_type;
static uint32_t last_result = RESULT_NOT_READY;
static bool capability_requested, raw_override, authority, pending_commit;
static bool maintenance_latched;
static bool settings_reconciled;
static uint32_t raw_applied_sequence, raw_sequence, neutral_output_rounds;
static bool pending_gate;
static bool commit_queued;
static uint8_t last_ack[12];
static bool last_ack_valid;
static uint16_t raw_motors[8];
static uint32_t raw_received_us;
static control_snapshot_t latest;
static uint32_t last_telemetry_us, last_stats_serial;
static uint16_t effective_motors[8];
static bool safety_recovery_pending, effective_valid;
static uint32_t closed_gate_session, closed_gate_sequence, closed_gate_crc;
static uint8_t closed_gate_ack[12];
static uint8_t staging[CONTROL_SETTINGS_WIRE_SIZE];
static uint32_t staging_generation, staging_digest, staging_received, staging_activity;
static bool staging_active;
static uint16_t pending_protocol, pending_speed;
static control_event_t commit_event;
static const uint motor_pins[8] = {MOTOR0_PIN_BASE,     MOTOR0_PIN_BASE + 1, MOTOR0_PIN_BASE + 2,
                                   MOTOR0_PIN_BASE + 3, MOTOR1_PIN_BASE,     MOTOR1_PIN_BASE + 1,
                                   MOTOR1_PIN_BASE + 2, MOTOR1_PIN_BASE + 3};

static uint32_t load_shared(const volatile uint32_t *value) {
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}
static void store_shared(volatile uint32_t *value, uint32_t next) {
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}
static uint32_t now_us(void) {
    return time_us_32();
}
static uint32_t age_us(uint32_t now, uint32_t then) {
    return now - then;
}
static void neutral(uint16_t motors[8]) {
    for (unsigned i = 0; i < 8; ++i) {
        motors[i] = 1000;
    }
}

static void send_frame(uint8_t type, uint32_t session, uint32_t sequence, const uint8_t *payload,
                       uint16_t length, bool priority) {
    control_frame_t frame = {0};
    frame.type = type;
    frame.session = session;
    frame.sequence = sequence;
    frame.length = length;
    if (length > 0) {
        memcpy(frame.payload, payload, length);
    }
    uint8_t encoded[CONTROL_FRAME_MAX];
    size_t count = control_frame_encode(encoded, sizeof(encoded), &frame);
    if (count > 0) {
        (void)usb_tx_packet(encoded, count, priority);
    }
}
static void encode_ack(uint8_t payload[12], uint32_t type, uint32_t result) {
    memset(payload, 0, 12);
    payload[0] = (uint8_t)type;
    payload[1] = (uint8_t)result;
    control_put_u32(payload + 4, active_generation);
    control_put_u32(payload + 8, active_digest);
}

static void send_uncached_ack(uint32_t session, uint32_t sequence, uint32_t type, uint32_t result) {
    uint8_t payload[12];
    encode_ack(payload, type, result);
    send_frame(FRAME_ACK, session, sequence, payload, sizeof(payload), true);
}

static void send_ack(uint32_t session, uint32_t sequence, uint32_t type, uint32_t result) {
    if (session == active_session && sequence == last_sequence) {
        last_result = result;
        encode_ack(last_ack, type, result);
        last_ack_valid = true;
    }
    send_uncached_ack(session, sequence, type, result);
}

void control_runtime_inhibit(void) {
    authority = false;
    raw_override = false;
    store_shared(&accepted_floor, last_sequence);
    store_shared(&safety_armed, 0);
    neutral(raw_motors);
}

bool control_runtime_output_permitted(void) {
    return effective_valid;
}

bool control_runtime_extended_active(void) {
    return active_session != 0;
}
bool control_runtime_maintenance_latched(void) {
    return maintenance_latched;
}

static bool event_push(const control_event_t *event) {
    uint32_t write = load_shared(&event_write);
    if (write - load_shared(&event_read) >= EVENT_COUNT) {
        store_shared(&queue_overflows, load_shared(&queue_overflows) + 1u);
        control_runtime_inhibit();
        return false;
    }
    events[write % EVENT_COUNT] = *event;
    store_shared(&event_write, write + 1u);
    return true;
}

static void applied_push(const control_event_t *event, uint32_t result) {
    uint32_t write = load_shared(&ack_write);
    if (write - load_shared(&ack_read) >= EVENT_COUNT) {
        /* Never block the controller for a host acknowledgement. A dropped
           result remains queryable through the committed snapshot. */
        return;
    }
    acknowledgements[write % EVENT_COUNT] = (application_ack_t){.session = event->session,
                                                                .sequence = event->sequence,
                                                                .type = event->type,
                                                                .generation = event->generation,
                                                                .digest = event->digest,
                                                                .result = result};
    store_shared(&ack_write, write + 1u);
}

static bool safety_interrupt(struct repeating_timer *timer) {
    (void)timer;
    uint32_t now = now_us();
    if (load_shared(&safety_armed) &&
        (age_us(now, load_shared(&core0_heartbeat)) > OUTPUT_TIMEOUT_US ||
         age_us(now, load_shared(&safety_output_us)) > OUTPUT_TIMEOUT_US ||
         age_us(now, load_shared(&safety_host_us)) > HOST_TIMEOUT_US)) {
        store_shared(&safety_latched, 1);
        for (unsigned i = 0; i < 8; ++i) {
            if (load_shared(&physical_protocol) == 0u) {
                /* PWM continues emitting neutral even if core0 is stuck. */
                pwm_set_gpio_level(motor_pins[i], pwm_translate_throttle(1000));
            } else {
                /* DShot is a waveform, not a static neutral level. Suppress
                   all output until normal code can safely send neutral frames. */
                gpio_set_outover(motor_pins[i], GPIO_OVERRIDE_LOW);
            }
        }
    }
    return true;
}

static void record_execution(execution_stats_t *stats, uint32_t duration) {
    stats->execution_sum += duration;
    if (duration > stats->maximum) {
        stats->maximum = duration;
    }
    if (duration < stats->minimum) {
        stats->minimum = duration;
    }
    unsigned bucket = 0;
    uint32_t limit = 125;
    while (bucket < 7u && duration > limit) {
        bucket++;
        limit *= 2u;
    }
    stats->histogram[bucket]++;
}

typedef struct {
    control_state_t state;
    control_snapshot_t snapshot;
    execution_stats_t stats;
    uint32_t session, generation, digest;
    uint32_t command_sequence, command_time, pressure_time, sample_time;
    uint32_t next_tick, last_init_attempt, stats_serial, overflow_baseline;
    uint64_t stats_started;
    bool pressure_healthy, stabilization, depth_hold, command_valid;
    bool sensor_initialized;
} core1_context_t;

static bool core1_pressure_healthy(const core1_context_t *ctx, uint32_t now) {
    return ctx->pressure_healthy && age_us(now, ctx->pressure_time) <= PRESSURE_TIMEOUT_US;
}

static bool core1_control_permitted(const core1_context_t *ctx, uint32_t now) {
    return ctx->command_valid && ctx->generation != 0 &&
           ctx->command_sequence > load_shared(&accepted_floor) &&
           age_us(now, ctx->command_time) <= HOST_TIMEOUT_US && !load_shared(&maintenance) &&
           !load_shared(&sensor_retry_requested) &&
           (!ctx->depth_hold || core1_pressure_healthy(ctx, now));
}

static bool core1_command_current(const control_event_t *event, uint32_t now) {
    return (event->flags & 4u) != 0 && age_us(now, event->received_us) <= HOST_TIMEOUT_US &&
           event->sequence > load_shared(&accepted_floor);
}

static void core1_apply_command(core1_context_t *ctx, const control_event_t *event, uint32_t now) {
    /* Reject stale depth input before the pure command can advance its PID or
       targets. Pressure never renews a lease or replays a rejected command. */
    bool depth = (event->flags & 2u) != 0;
    ctx->command_valid = core1_command_current(event, now) && ctx->generation != 0 &&
                         !load_shared(&maintenance) && !load_shared(&sensor_retry_requested) &&
                         (!depth || core1_pressure_healthy(ctx, now));
    if (!ctx->command_valid) {
        return;
    }
    ctx->stabilization = (event->flags & 1u) != 0;
    ctx->depth_hold = depth;
    control_command(&ctx->state, event->values, event->values[8], ctx->stabilization, depth);
    ctx->command_sequence = event->sequence;
    ctx->command_time = event->received_us;
    if (depth) {
        ctx->stats.depth++;
    }
}

static void core1_apply_event(core1_context_t *ctx, const control_event_t *event, uint32_t now) {
    switch (event->type) {
    case MSG_COMMIT:
        if (event->sequence != load_shared(&commit_authorized_sequence)) {
            break;
        }
        control_apply_settings(&ctx->state, &event->settings);
        ctx->generation = event->generation;
        ctx->digest = event->digest;
        ctx->snapshot.committed_sequence = event->sequence;
        applied_push(event, RESULT_APPLIED);
        break;
    case MSG_PRESSURE:
        ctx->pressure_healthy = event->flags != 0;
        ctx->pressure_time = event->received_us;
        control_pressure(&ctx->state, event->values[0], event->values[1]);
        break;
    case MSG_ATTITUDE: {
        bool ok = control_set_attitude(&ctx->state, event->values);
        applied_push(event, ok ? RESULT_APPLIED : RESULT_INVALID);
        break;
    }
    case MSG_DEPTH:
        control_set_depth(&ctx->state, event->values[0]);
        applied_push(event, RESULT_APPLIED);
        break;
    case MSG_CONTROL:
        core1_apply_command(ctx, event, now);
        break;
    default:
        break;
    }
}

static void core1_process_events(core1_context_t *ctx, uint32_t now, bool fresh) {
    uint32_t read = load_shared(&event_read);
    /* Bound each pass to the queue observed at entry, even if USB keeps writing. */
    uint32_t write = load_shared(&event_write);
    while (read != write) {
        const control_event_t *event = &events[read % EVENT_COUNT];
        bool matching = event->session == ctx->session && ctx->session != 0;
        bool command = matching && event->type == MSG_CONTROL;
        if (command && !core1_command_current(event, now)) {
            /* Invalid input has no source step to preserve. In particular, an
               IMU outage must not strand a COMMIT behind inhibited commands. */
            ctx->command_valid = false;
            store_shared(&event_read, ++read);
            continue;
        }
        if (command && !fresh) {
            break;
        }
        if (matching) {
            core1_apply_event(ctx, event, now);
        }
        store_shared(&event_read, ++read);
        /* One command per fresh sample/allocation; never batch source steps. */
        if (command) {
            break;
        }
    }
}

static void core1_session(core1_context_t *ctx) {
    uint32_t requested = load_shared(&shared_session);
    if (requested == ctx->session) {
        return;
    }
    control_new_session(&ctx->state);
    ctx->session = requested;
    ctx->generation = ctx->digest = ctx->command_sequence = ctx->command_time = 0;
    ctx->pressure_time = 0;
    ctx->snapshot.committed_sequence = 0;
    ctx->command_valid = ctx->stabilization = ctx->depth_hold = ctx->pressure_healthy = false;
}

static uint32_t core1_wait_tick(core1_context_t *ctx) {
    uint32_t now = now_us();
    if ((int32_t)(now - ctx->next_tick) < 0) {
        busy_wait_until(from_us_since_boot(time_us_64() + (ctx->next_tick - now)));
        now = now_us();
    }
    ctx->next_tick += PERIOD_US;
    if ((int32_t)(now - ctx->next_tick) >= 0) {
        uint32_t skipped = ((now - ctx->next_tick) / PERIOD_US) + 1u;
        ctx->stats.missed += skipped;
        ctx->next_tick += skipped * PERIOD_US;
    }
    store_shared(&core1_heartbeat, now);
    if (age_us(now, load_shared(&core0_heartbeat)) < 100000u || load_shared(&maintenance)) {
        watchdog_update();
    }
    return now;
}

static void core1_retry_sensor(core1_context_t *ctx, uint32_t now) {
    if (ctx->sensor_initialized || load_shared(&output_permitted) ||
        age_us(now, ctx->last_init_attempt) <= 1000000u) {
        return;
    }
    /* A disarmed safety timer also describes permitted neutral thrust. Require
       core0 to physically service inhibited neutral output before blocking SPI. */
    store_shared(&sensor_retry_requested, 1);
    if (!load_shared(&sensor_retry_inhibited)) {
        return;
    }
    ctx->last_init_attempt = now;
    ctx->command_valid = false;
    ctx->sensor_initialized = bmi270_sensor_init();
    if (!ctx->sensor_initialized) {
        ctx->stats.errors++;
    }
    store_shared(&sensor_retry_requested, 0);
    store_shared(&sensor_retry_inhibited, 0);
}

static bool core1_read_sensor(core1_context_t *ctx, control_sample_t *sample, float *temperature) {
    if (!ctx->sensor_initialized) {
        return false;
    }
    bool fresh = bmi270_sensor_read(sample->accel, sample->gyro, temperature);
    bmi270_sensor_diagnostics_t diagnostics;
    bmi270_sensor_get_diagnostics(&diagnostics);
    ctx->sensor_initialized = diagnostics.initialized;
    if (!fresh && diagnostics.result != BMI270_SENSOR_NOT_READY) {
        ctx->stats.errors++;
    }
    return fresh;
}

static void core1_sample(core1_context_t *ctx, const control_sample_t *sample, float temperature,
                         uint32_t now) {
    float dt = ctx->sample_time == 0 ? 0.002f : (float)age_us(now, ctx->sample_time) * 0.000001f;
    ctx->sample_time = now;
    ctx->snapshot.sample = *sample;
    ctx->snapshot.temperature = temperature;
    if (core1_control_permitted(ctx, now)) {
        control_step(&ctx->state, sample, dt, &ctx->snapshot.output);
        if (ctx->stabilization) {
            ctx->stats.pid++;
        }
    } else {
        control_observe(&ctx->state, sample, dt, &ctx->snapshot.output);
    }
    ctx->stats.ahrs++;
}

static void core1_snapshot(core1_context_t *ctx, uint32_t now) {
    bool imu = ctx->sensor_initialized && ctx->sample_time != 0 &&
               age_us(now, ctx->sample_time) <= OUTPUT_TIMEOUT_US;
    bool pressure = core1_pressure_healthy(ctx, now);
    bool allowed = imu && core1_control_permitted(ctx, now);
    control_snapshot_t *snapshot = &ctx->snapshot;
    bmi270_sensor_get_diagnostics(&snapshot->sensor);
    snapshot->health = (imu ? 1u : 0u) | (pressure ? 2u : 0u) | (allowed ? 4u : 0u);
    if (!allowed) {
        neutral(snapshot->output.motors);
    }
    snapshot->session = ctx->session;
    snapshot->sequence = ctx->command_sequence;
    snapshot->command_us = ctx->command_time;
    /* Publication of an old allocation must not renew the output freshness. */
    snapshot->completed_us = ctx->sample_time;
    snapshot->generation = ctx->generation;
    snapshot->digest = ctx->digest;
}

static void core1_stats(core1_context_t *ctx, uint32_t start) {
    uint32_t now = now_us();
    uint32_t host_age = ctx->command_time ? age_us(now, ctx->command_time) : 0;
    if (host_age > ctx->stats.host_max) {
        ctx->stats.host_max = host_age;
    }
    uint32_t output_age = age_us(now, load_shared(&safety_output_us));
    if (load_shared(&safety_armed) && output_age > ctx->stats.output_max) {
        ctx->stats.output_max = output_age;
    }
    record_execution(&ctx->stats, age_us(now, start));
    uint64_t finished = time_us_64();
    uint64_t elapsed = finished - ctx->stats_started;
    if (elapsed >= 5000000u) {
        uint32_t overflows = load_shared(&queue_overflows);
        ctx->stats.elapsed_us = elapsed;
        ctx->stats.overflows = overflows - ctx->overflow_baseline;
        ctx->overflow_baseline = overflows;
        ctx->snapshot.stats = ctx->stats;
        ctx->snapshot.stats_serial = ++ctx->stats_serial;
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        ctx->stats.minimum = UINT32_MAX;
        ctx->stats_started = finished;
    }
}

static void core1_publish(const core1_context_t *ctx) {
    uint32_t write = load_shared(&snapshot_write);
    if (write - load_shared(&snapshot_read) < SNAPSHOT_COUNT) {
        snapshots[write % SNAPSHOT_COUNT] = ctx->snapshot;
        store_shared(&snapshot_write, write + 1u);
    }
}

static void core1_main(void) {
    static core1_context_t ctx;
    control_init(&ctx.state);
    ctx.snapshot.output.current_q[3] = 1.0f;
    ctx.snapshot.output.desired_q[3] = 1.0f;
    ctx.sensor_initialized = bmi270_sensor_init();
    ctx.next_tick = now_us();
    ctx.last_init_attempt = ctx.next_tick;
    ctx.stats.minimum = UINT32_MAX;
    ctx.stats.errors = ctx.sensor_initialized ? 0u : 1u;
    ctx.stats_started = time_us_64();
    for (;;) {
        uint32_t start = core1_wait_tick(&ctx);
        core1_session(&ctx);
        core1_retry_sensor(&ctx, start);
        control_sample_t sample;
        float temperature;
        bool fresh = core1_read_sensor(&ctx, &sample, &temperature);
        uint32_t now = now_us();
        core1_process_events(&ctx, now, fresh);
        if (fresh) {
            core1_sample(&ctx, &sample, temperature, now);
        }
        core1_snapshot(&ctx, now);
        core1_stats(&ctx, start);
        core1_publish(&ctx);
    }
}

void control_runtime_capabilities(uint8_t request_id) {
    static const char identity[] = CONTROL_BUILD_IDENTITY;
    uint8_t payload[12 + sizeof(identity) - 1];
    control_put_u16(payload, 1);
    control_put_u16(payload + 2, CONTROL_MAX_PAYLOAD);
    control_put_u16(payload + 4, CONTROL_SETTINGS_WIRE_SIZE);
    control_put_u16(payload + 6, 8);
    control_put_u32(payload + 8, 0x1f);
    memcpy(payload + 12, identity, sizeof(identity) - 1);
    capability_requested = true;
    send_frame(FRAME_CAPS, 0, request_id, payload, sizeof(payload), true);
}

static bool motors_neutral(const uint16_t motors[8]) {
    for (unsigned i = 0; i < 8; ++i) {
        if (motors[i] != 1000) {
            return false;
        }
    }
    return true;
}

static void begin_session(const control_frame_t *frame) {
    if (frame->session != 0 && frame->session == safety_failed_session) {
        send_uncached_ack(frame->session, frame->sequence, frame->type, RESULT_NOT_READY);
        return;
    }
    if (!capability_requested || frame->session == 0 || frame->length != 0 ||
        frame->sequence != 1 || safety_recovery_pending || callbacks.maintenance_active() ||
        (effective_valid && !motors_neutral(effective_motors))) {
        send_ack(frame->session, frame->sequence, frame->type, RESULT_BUSY);
        return;
    }
    if (frame->session == active_session) {
        /* A lost HELLO ACK is retryable, but cannot reset an established session. */
        send_ack(frame->session, frame->sequence, frame->type,
                 last_sequence == 1 ? RESULT_APPLIED : RESULT_STALE);
        return;
    }
    control_runtime_inhibit();
    active_session = frame->session;
    settings_reconciled = false;
    closed_gate_session = closed_gate_sequence = closed_gate_crc = 0;
    active_generation = active_digest = active_commit_sequence = 0;
    last_sequence = 1;
    pending_sequence = 0;
    pending_commit = pending_gate = staging_active = commit_queued = false;
    store_shared(&commit_authorized_sequence, 0);
    store_shared(&accepted_floor, 1);
    store_shared(&shared_session, active_session);
    send_ack(active_session, 1, MSG_HELLO, RESULT_APPLIED);
}

static bool floats_decode(float *out, const uint8_t *payload, unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        out[i] = control_get_f32(payload + (i * 4u));
        if (!isfinite(out[i])) {
            return false;
        }
    }
    return true;
}

static uint32_t settings_begin(const control_frame_t *frame, uint32_t received) {
    if (frame->length != 12 || control_get_u32(frame->payload + 4) != CONTROL_SETTINGS_WIRE_SIZE ||
        control_get_u32(frame->payload) == 0 || pending_commit) {
        return RESULT_INVALID;
    }
    staging_generation = control_get_u32(frame->payload);
    staging_digest = control_get_u32(frame->payload + 8);
    if (staging_generation == active_generation && staging_digest != active_digest) {
        return RESULT_INVALID;
    }
    staging_received = 0;
    staging_active = true;
    staging_activity = received;
    return RESULT_STAGED;
}

static uint32_t settings_abort(const control_frame_t *frame) {
    if (frame->length != 0 || pending_commit) {
        return RESULT_BUSY;
    }
    staging_active = false;
    return RESULT_APPLIED;
}

static uint32_t settings_chunk(const control_frame_t *frame, uint32_t received) {
    if (frame->length <= 4) {
        return RESULT_INVALID;
    }
    uint32_t offset = control_get_u32(frame->payload);
    uint32_t count = frame->length - 4u;
    if (offset > CONTROL_SETTINGS_WIRE_SIZE || count > CONTROL_SETTINGS_WIRE_SIZE - offset) {
        return RESULT_INVALID;
    }
    if (offset < staging_received) {
        if (offset + count > staging_received ||
            memcmp(staging + offset, frame->payload + 4, count) != 0) {
            return RESULT_INVALID;
        }
        staging_activity = received;
        return RESULT_STAGED;
    }
    if (offset != staging_received) {
        return RESULT_INVALID;
    }
    memcpy(staging + offset, frame->payload + 4, count);
    staging_received += count;
    staging_activity = received;
    return RESULT_STAGED;
}

static uint32_t settings_commit(const control_frame_t *frame, uint32_t received) {
    if (frame->type != MSG_COMMIT || frame->length != 8 ||
        control_get_u32(frame->payload) != staging_generation ||
        control_get_u32(frame->payload + 4) != staging_digest ||
        staging_received != CONTROL_SETTINGS_WIRE_SIZE ||
        control_crc32c(staging, sizeof(staging)) != staging_digest ||
        !control_settings_decode(staging, sizeof(staging), &commit_event.settings,
                                 &pending_protocol, &pending_speed)) {
        return RESULT_INVALID;
    }
    commit_event.type = MSG_COMMIT;
    commit_event.session = active_session;
    commit_event.sequence = frame->sequence;
    commit_event.generation = staging_generation;
    commit_event.digest = staging_digest;
    commit_event.received_us = received;
    /* Output hardware may transition before pure settings, but it has no thrust
       authority throughout. Only the final core1 ACK commits the generation. */
    control_runtime_inhibit();
    if (!callbacks.request_protocol(pending_protocol, pending_speed)) {
        return RESULT_BUSY;
    }
    settings_reconciled = false;
    pending_commit = true;
    commit_queued = false;
    store_shared(&commit_authorized_sequence, frame->sequence);
    pending_sequence = frame->sequence;
    pending_type = MSG_COMMIT;
    staging_activity = received;
    return UINT32_MAX;
}

static uint32_t settings_request(const control_frame_t *frame, uint32_t received) {
    /* Active metadata is diagnostic state, including during recovery. */
    if (frame->type == MSG_QUERY) {
        return frame->length == 0 ? RESULT_APPLIED : RESULT_INVALID;
    }
    if (callbacks.maintenance_active() || callbacks.recovery_required()) {
        return RESULT_BUSY;
    }
    switch (frame->type) {
    case MSG_BEGIN:
        return settings_begin(frame, received);
    case MSG_ABORT:
        return settings_abort(frame);
    case MSG_COMMIT:
        if (frame->length == 8 && active_generation != 0 && settings_reconciled &&
            control_get_u32(frame->payload) == active_generation &&
            control_get_u32(frame->payload + 4) == active_digest) {
            return RESULT_APPLIED;
        }
        break;
    default:
        break;
    }
    if (!staging_active || age_us(received, staging_activity) > SETTINGS_TIMEOUT_US) {
        staging_active = false;
        return RESULT_NOT_READY;
    }
    return frame->type == MSG_CHUNK ? settings_chunk(frame, received)
                                    : settings_commit(frame, received);
}

static uint32_t raw_request(const control_frame_t *frame, uint32_t received) {
    if (frame->length != 16) {
        return RESULT_INVALID;
    }
    uint16_t values[8];
    for (unsigned i = 0; i < 8; ++i) {
        values[i] = control_get_u16(frame->payload + (i * 2u));
        if (values[i] > 2000) {
            return RESULT_INVALID;
        }
    }
    bool is_neutral = motors_neutral(values);
    if (!is_neutral &&
        (active_generation == 0 || !settings_reconciled || callbacks.maintenance_active() ||
         callbacks.recovery_required() || maintenance_latched)) {
        return RESULT_NOT_READY;
    }
    control_runtime_inhibit();
    memcpy(raw_motors, values, sizeof(values));
    raw_override = authority = true;
    raw_received_us = received;
    raw_sequence = frame->sequence;
    raw_applied_sequence = neutral_output_rounds = 0;
    return UINT32_MAX;
}

static uint32_t gate_request(const control_frame_t *frame, uint32_t received) {
    if (frame->length != 0 || !raw_override || !motors_neutral(raw_motors) ||
        age_us(received, raw_received_us) > HOST_TIMEOUT_US) {
        return RESULT_NOT_READY;
    }
    authority = false;
    maintenance_latched = true;
    store_shared(&accepted_floor, last_sequence);
    pending_gate = true;
    pending_sequence = frame->sequence;
    pending_type = 0x25u;
    return UINT32_MAX;
}

static uint32_t decode_command(const control_frame_t *frame, control_event_t *event) {
    if (frame->length != 40 || !floats_decode(event->values, frame->payload, 8)) {
        return RESULT_INVALID;
    }
    for (unsigned i = 0; i < 8; ++i) {
        if (fabsf(event->values[i]) > 1.0f) {
            return RESULT_INVALID;
        }
    }
    event->values[8] = control_get_f32(frame->payload + 32);
    event->flags = control_get_u32(frame->payload + 36);
    if ((event->flags & ~7u) != 0) {
        return RESULT_INVALID;
    }
    if (active_generation == 0 || !settings_reconciled || callbacks.maintenance_active() ||
        callbacks.recovery_required()) {
        return RESULT_NOT_READY;
    }
    if ((event->flags & 4u) == 0) {
        control_runtime_inhibit();
    } else {
        authority = true;
        raw_override = false;
        maintenance_latched = false;
    }
    return RESULT_APPLIED;
}

static uint32_t command_request(const control_frame_t *frame, uint32_t received) {
    if (frame->type == MSG_RAW) {
        return raw_request(frame, received);
    }
    if (frame->type == 0x25u) {
        return gate_request(frame, received);
    }
    static control_event_t event;
    memset(&event, 0, sizeof(event));
    event.type = frame->type;
    event.session = active_session;
    event.sequence = frame->sequence;
    event.received_us = received;
    event.generation = active_generation;
    event.digest = active_digest;
    if (frame->type == MSG_PRESSURE) {
        if (frame->length != 12 || !floats_decode(event.values, frame->payload, 2)) {
            return RESULT_INVALID;
        }
        event.flags = control_get_u32(frame->payload + 8);
        if (event.flags > 1) {
            return RESULT_INVALID;
        }
    } else if (frame->type == MSG_CONTROL) {
        uint32_t result = decode_command(frame, &event);
        if (result != RESULT_APPLIED) {
            return result;
        }
    } else if (frame->type == MSG_ATTITUDE || frame->type == MSG_DEPTH) {
        unsigned count = frame->type == MSG_ATTITUDE ? 4u : 1u;
        if (frame->length != count * 4u || !floats_decode(event.values, frame->payload, count)) {
            return RESULT_INVALID;
        }
        if (active_generation == 0 || callbacks.recovery_required() ||
            callbacks.maintenance_active()) {
            return RESULT_NOT_READY;
        }
    } else {
        return RESULT_UNSUPPORTED;
    }
    if (!event_push(&event)) {
        return RESULT_BUSY;
    }
    if (frame->type == MSG_ATTITUDE || frame->type == MSG_DEPTH) {
        pending_sequence = frame->sequence;
        pending_type = frame->type;
    }
    return UINT32_MAX;
}

void control_runtime_receive_at(const uint8_t *packet, size_t length, uint32_t received) {
    static control_frame_t frame;
    if (!control_frame_decode(packet, length, &frame)) {
        control_runtime_inhibit();
        return;
    }
    if (frame.type == MSG_HELLO) {
        begin_session(&frame);
        return;
    }
    if (frame.type == 0x25u && frame.session == closed_gate_session &&
        frame.sequence == closed_gate_sequence &&
        control_crc32c(packet, length - 4u) == closed_gate_crc) {
        send_frame(FRAME_ACK, frame.session, frame.sequence, closed_gate_ack,
                   sizeof(closed_gate_ack), true);
        return;
    }
    if (frame.session == 0 || frame.session != active_session) {
        send_ack(frame.session, frame.sequence, frame.type, RESULT_NOT_READY);
        return;
    }
    uint32_t request_crc = control_crc32c(packet, length - 4u);
    if (frame.sequence == last_sequence) {
        if (request_crc != last_request_crc) {
            send_uncached_ack(active_session, frame.sequence, frame.type, RESULT_INVALID);
        } else if (pending_sequence != frame.sequence && last_ack_valid) {
            send_frame(FRAME_ACK, active_session, frame.sequence, last_ack, sizeof(last_ack), true);
        }
        return;
    }
    if (frame.sequence == 0 || frame.sequence < last_sequence) {
        send_ack(active_session, frame.sequence, frame.type, RESULT_STALE);
        return;
    }
    if (pending_sequence != 0) {
        send_ack(active_session, frame.sequence, frame.type, RESULT_BUSY);
        return;
    }
    last_sequence = frame.sequence;
    last_request_crc = request_crc;
    last_result = RESULT_APPLIED;
    last_ack_valid = false;
    uint32_t result = frame.type >= MSG_BEGIN && frame.type <= MSG_QUERY
                          ? settings_request(&frame, received)
                          : command_request(&frame, received);
    if (result != UINT32_MAX) {
        if (result != RESULT_APPLIED && result != RESULT_STAGED && frame.type == MSG_CONTROL) {
            control_runtime_inhibit();
        }
        send_ack(active_session, frame.sequence, frame.type, result);
    }
}

void control_runtime_receive(const uint8_t *packet, size_t length) {
    control_runtime_receive_at(packet, length, now_us());
}

void control_runtime_legacy_input_at(const uint16_t motors[8], uint32_t received) {
    if ((active_session != 0 || maintenance_latched) && !motors_neutral(motors)) {
        control_runtime_inhibit();
        return;
    }
    control_runtime_inhibit();
    memcpy(raw_motors, motors, sizeof(raw_motors));
    raw_received_us = received;
    raw_override = authority = true;
    raw_sequence = raw_applied_sequence = neutral_output_rounds = 0;
}

void control_runtime_legacy_input(const uint16_t motors[8]) {
    control_runtime_legacy_input_at(motors, now_us());
}

void control_runtime_get_motors(uint16_t motors[8]) {
    uint32_t now = now_us();
    neutral(motors);
    bool ready = callbacks.outputs_initialized() && !callbacks.maintenance_active() &&
                 !callbacks.recovery_required() && !pending_commit &&
                 !load_shared(&sensor_retry_requested);
    uint32_t host_time = raw_override ? raw_received_us : latest.command_us;
    uint32_t output_time = raw_override ? now : latest.completed_us;
    bool valid = authority && ready && age_us(now, host_time) <= HOST_TIMEOUT_US &&
                 age_us(now, output_time) <= OUTPUT_TIMEOUT_US;
    if (raw_override && maintenance_latched && !motors_neutral(raw_motors)) {
        valid = false;
    }
    if (!raw_override) {
        valid = valid && latest.session == active_session && active_session != 0 &&
                active_generation != 0 && settings_reconciled &&
                latest.generation == active_generation &&
                latest.sequence > load_shared(&accepted_floor) && (latest.health & 4u) != 0;
    }
    effective_valid = valid;
    store_shared(&output_permitted, valid ? 1u : 0u);
    if (valid) {
        memcpy(motors, raw_override ? raw_motors : latest.output.motors, sizeof(raw_motors));
    }
    memcpy(effective_motors, motors, sizeof(effective_motors));
    store_shared(&physical_protocol, callbacks.output_protocol());
    store_shared(&safety_host_us, host_time);
    store_shared(&safety_output_us, output_time);
    store_shared(&safety_armed, valid && !motors_neutral(motors) ? 1u : 0u);
}

void control_runtime_output_serviced(void) {
    if (load_shared(&sensor_retry_requested) && !effective_valid &&
        motors_neutral(effective_motors)) {
        store_shared(&sensor_retry_inhibited, 1);
    }
    if (raw_override && motors_neutral(effective_motors) && motors_neutral(raw_motors)) {
        neutral_output_rounds++;
        uint32_t needed = callbacks.output_protocol() == 0 ? 1u : 4u;
        if (neutral_output_rounds >= needed) {
            raw_applied_sequence = raw_sequence;
        }
    }
    if (safety_recovery_pending && motors_neutral(effective_motors)) {
        for (unsigned i = 0; i < 8; ++i) {
            gpio_set_outover(motor_pins[i], GPIO_OVERRIDE_NORMAL);
        }
        safety_recovery_pending = false;
    }
}

static void publish_telemetry(uint32_t now) {
    if (active_session == 0 || age_us(now, last_telemetry_us) < 16666u) {
        return;
    }
    last_telemetry_us = now;
    uint8_t payload[84] = {0};
    control_put_u64(payload, time_us_64());
    for (unsigned i = 0; i < 4; ++i) {
        control_put_f32(payload + 8 + (i * 4u), latest.output.current_q[i]);
        control_put_f32(payload + 24 + (i * 4u), latest.output.desired_q[i]);
    }
    control_put_f32(payload + 40, latest.output.desired_depth);
    control_put_u32(payload + 44, active_generation);
    control_put_u32(payload + 48, latest.sequence);
    uint32_t health = latest.health & 3u;
    if (effective_valid) {
        health |= 4u;
    }
    if (raw_override) {
        health |= 8u;
    }
    control_put_u32(payload + 52, health);
    control_put_u32(payload + 56, age_us(now, raw_override ? raw_received_us : latest.command_us));
    control_put_u32(payload + 60, age_us(now, latest.completed_us));
    for (unsigned i = 0; i < 8; ++i) {
        control_put_u16(payload + 64 + (i * 2u), effective_motors[i]);
    }
    control_put_u32(payload + 80,
                    raw_override || !effective_valid ? 0u : latest.output.work_percent);
    send_frame(FRAME_ATTITUDE, active_session, ++telemetry_sequence, payload, sizeof(payload),
               false);
    uint8_t imu[28];
    for (unsigned i = 0; i < 3; ++i) {
        control_put_f32(imu + (i * 4u), latest.sample.accel[i]);
        control_put_f32(imu + 12 + (i * 4u), latest.sample.gyro[i]);
    }
    control_put_f32(imu + 24, latest.temperature);
    send_frame(FRAME_IMU, active_session, ++telemetry_sequence, imu, sizeof(imu), false);
}

static void publish_sensor_diagnostics(void) {
    const bmi270_sensor_diagnostics_t *sensor = &latest.sensor;
    log_infof("BMI270 chip=%02x init=%u internal=%02x err=%02x status=%02x cfg=%02x/%02x/%02x/%02x "
              "pwr=%02x/%02x time=%06lx samples=%lu result=%u bosch=%d bus_timeouts=%lu",
              sensor->chip_id, sensor->initialized ? 1u : 0u, sensor->internal_status,
              sensor->error, sensor->status, sensor->config[0], sensor->config[1],
              sensor->config[2], sensor->config[3], sensor->power[0], sensor->power[1],
              (unsigned long)sensor->sensor_time, (unsigned long)sensor->samples,
              (unsigned)sensor->result, (int)sensor->bosch_result,
              (unsigned long)sensor->bus_timeouts);
}

static void publish_stats(void) {
    if (latest.stats_serial == 0 || latest.stats_serial == last_stats_serial) {
        return;
    }
    last_stats_serial = latest.stats_serial;
    publish_sensor_diagnostics();
    const execution_stats_t *stats = &latest.stats;
    uint32_t loops = 0;
    for (unsigned i = 0; i < 8; ++i) {
        loops += stats->histogram[i];
    }
    uint32_t average = loops ? (uint32_t)(stats->execution_sum / loops) : 0;
    uint32_t p99_count = (loops * 99u + 99u) / 100u;
    uint32_t p99 = 125;
    uint32_t cumulative = 0;
    for (unsigned i = 0; i < 8; ++i) {
        cumulative += stats->histogram[i];
        if (i == 7u) {
            /* The overflow bucket is unbounded: only the observed maximum
               supplies a truthful upper bound for its percentile. */
            p99 = stats->maximum;
            break;
        }
        if (cumulative >= p99_count) {
            break;
        }
        p99 *= 2u;
    }
    uint8_t payload[48];
    control_put_u64(payload, stats->elapsed_us);
    control_put_u32(payload + 8, stats->ahrs);
    control_put_u32(payload + 12, stats->pid);
    control_put_u32(payload + 16, stats->depth);
    control_put_u32(payload + 20, stats->missed);
    control_put_u32(payload + 24, average);
    control_put_u32(payload + 28, stats->maximum);
    control_put_u32(payload + 32, stats->errors);
    control_put_u32(payload + 36, stats->overflows);
    control_put_u32(payload + 40, stats->host_max);
    control_put_u32(payload + 44, stats->output_max);
    if (active_session != 0) {
        send_frame(FRAME_STATS, active_session, ++telemetry_sequence, payload, sizeof(payload),
                   false);
    }
    double seconds = (double)stats->elapsed_us / 1000000.0;
    log_infof("Pico control: AHRS %.2fHz PID %.2fHz depth %.2fHz; us min/avg/max=%u/%u/%u p99<=%u "
              "misses=%u sensor_errors=%u",
              (double)stats->ahrs / seconds, (double)stats->pid / seconds,
              (double)stats->depth / seconds, stats->minimum, average, stats->maximum, p99,
              stats->missed, stats->errors);
    log_infof(
        "Pico safety: host_max=%uus output_max=%uus queue_overflows=%u USB_drops=%u; duty=%.1f%%",
        stats->host_max, stats->output_max, stats->overflows, usb_tx_dropped(),
        100.0 * (double)stats->execution_sum / (double)stats->elapsed_us);
}

static void reconcile_settings(void) {
    if (active_commit_sequence == commit_event.sequence &&
        active_generation == commit_event.generation && active_digest == commit_event.digest &&
        callbacks.protocol_ready(pending_protocol, pending_speed) &&
        !callbacks.maintenance_active() && !callbacks.recovery_required()) {
        settings_reconciled = true;
    }
}

static void consume_snapshots(void) {
    uint32_t read = load_shared(&snapshot_read);
    while (read != load_shared(&snapshot_write)) {
        latest = snapshots[read % SNAPSHOT_COUNT];
        read++;
        store_shared(&snapshot_read, read);
    }
    /* A timed-out ACK may still have applied. Never let an older snapshot
       overwrite a newer ACK; QUERY always reports observed committed state. */
    if (latest.session == active_session && latest.committed_sequence > active_commit_sequence) {
        active_generation = latest.generation;
        active_digest = latest.digest;
        active_commit_sequence = latest.committed_sequence;
        reconcile_settings();
        if (latest.committed_sequence == last_sequence) {
            last_result = RESULT_APPLIED;
            encode_ack(last_ack, MSG_COMMIT, RESULT_APPLIED);
            last_ack_valid = true;
        }
    }
}

static void finish_commit(uint32_t result) {
    send_ack(active_session, pending_sequence, MSG_COMMIT, result);
    pending_sequence = 0;
    pending_commit = commit_queued = staging_active = false;
    store_shared(&commit_authorized_sequence, 0);
}

static void service_commit(uint32_t now) {
    if (!pending_commit) {
        return;
    }
    if (active_commit_sequence == pending_sequence &&
        active_generation == commit_event.generation && active_digest == commit_event.digest) {
        finish_commit(RESULT_APPLIED);
        return;
    }
    if (age_us(now, commit_event.received_us) > 7000000u || callbacks.recovery_required()) {
        /* Also cancels a queued event not yet consumed. If core1 was already
           applying it, its eventual snapshot/ACK reconciles the outcome. */
        finish_commit(RESULT_NOT_READY);
        settings_reconciled = false;
        control_runtime_inhibit();
        return;
    }
    if (!commit_queued && callbacks.protocol_ready(pending_protocol, pending_speed)) {
        commit_queued = event_push(&commit_event);
    }
}

static void consume_acknowledgements(void) {
    uint32_t read = load_shared(&ack_read);
    while (read != load_shared(&ack_write)) {
        application_ack_t ack = acknowledgements[read % EVENT_COUNT];
        read++;
        store_shared(&ack_read, read);
        if (ack.session != active_session) {
            continue;
        }
        if (ack.type == MSG_COMMIT && ack.result == RESULT_APPLIED &&
            ack.sequence >= active_commit_sequence) {
            active_generation = ack.generation;
            active_digest = ack.digest;
            active_commit_sequence = ack.sequence;
            reconcile_settings();
            if (ack.sequence == last_sequence) {
                last_result = ack.result;
                encode_ack(last_ack, ack.type, ack.result);
                last_ack_valid = true;
            }
        }
        if (ack.sequence != pending_sequence) {
            continue;
        }
        if (ack.type == MSG_COMMIT) {
            finish_commit(ack.result);
        } else {
            send_ack(ack.session, ack.sequence, ack.type, ack.result);
            pending_sequence = 0;
        }
    }
}

static void recover_safety_latch(void) {
    /* Arrival time cannot be reconstructed after masked USB interrupts. A new
       sequence alone is insufficient: poison the session and require HELLO,
       a fresh settings commit and fresh CONTROL before restoring authority. */
    control_runtime_inhibit();
    if (pending_sequence != 0) {
        send_ack(active_session, pending_sequence, pending_type, RESULT_NOT_READY);
    }
    pending_sequence = 0;
    pending_commit = pending_gate = commit_queued = staging_active = false;
    store_shared(&commit_authorized_sequence, 0);
    maintenance_latched = true;
    safety_failed_session = active_session;
    settings_reconciled = false;
    active_session = active_generation = active_digest = active_commit_sequence = 0;
    closed_gate_session = closed_gate_sequence = closed_gate_crc = 0;
    last_ack_valid = false;
    store_shared(&shared_session, 0);
    store_shared(&safety_latched, 0);
    safety_recovery_pending = true;
}

static void service_gate(void) {
    uint32_t needed = callbacks.output_protocol() == 0 ? 1u : 4u;
    bool neutral_serviced = raw_applied_sequence == raw_sequence && neutral_output_rounds >= needed;
    if (pending_gate &&
        (neutral_serviced || callbacks.recovery_required() || !callbacks.outputs_initialized())) {
        /* A recovery/uninitialized device already has no motor waveform authority. */
        store_shared(&safety_armed, 0);
        maintenance_latched = true;
        closed_gate_session = active_session;
        closed_gate_sequence = pending_sequence;
        closed_gate_crc = last_request_crc;
        memset(closed_gate_ack, 0, sizeof(closed_gate_ack));
        closed_gate_ack[0] = 0x25u;
        control_put_u32(closed_gate_ack + 4, active_generation);
        control_put_u32(closed_gate_ack + 8, active_digest);
        send_ack(active_session, pending_sequence, 0x25u, RESULT_APPLIED);
        pending_sequence = 0;
        pending_gate = false;
        control_runtime_inhibit();
        active_session = 0;
        active_generation = active_digest = 0;
        store_shared(&shared_session, 0);
    }
}

void control_runtime_service(void) {
    uint32_t now = now_us();
    store_shared(&core0_heartbeat, now);
    bool suspended = callbacks.maintenance_active() || callbacks.recovery_required() ||
                     pending_commit || load_shared(&sensor_retry_requested);
    store_shared(&maintenance, suspended ? 1u : 0u);
    if (suspended) {
        control_runtime_inhibit();
    }
    if (!authority || age_us(now, load_shared(&core1_heartbeat)) < 100000u) {
        watchdog_update();
    }
    if (load_shared(&safety_latched)) {
        recover_safety_latch();
    }
    consume_snapshots();
    consume_acknowledgements();
    service_commit(now);
    service_gate();
    if (staging_active && !pending_commit && age_us(now, staging_activity) > SETTINGS_TIMEOUT_US) {
        staging_active = false;
    }
    if (!callbacks.maintenance_active()) {
        publish_telemetry(now);
        publish_stats();
    }
    usb_tx_service();
}

void control_runtime_init(const control_runtime_hooks_t *hooks) {
    callbacks = *hooks;
    neutral(raw_motors);
    neutral(effective_motors);
    latest.output.current_q[3] = 1.0f;
    latest.output.desired_q[3] = 1.0f;
    core0_heartbeat = core1_heartbeat = now_us();
    watchdog_enable(1000, true);
    multicore_launch_core1_with_stack(core1_main, core1_stack, sizeof(core1_stack));
    if (!add_repeating_timer_us(-1000, safety_interrupt, NULL, &safety_timer)) {
        log_error("Cannot allocate motor safety timer; refusing output");
        while (true) {
            tight_loop_contents();
        }
    }
}
