/* Compile the actual runtime/parser/RX/TX with deterministic hardware boundaries.
 * Each scenario gets a fresh process, so production static initialization is preserved. */
#include "control/controller.h"
#include "control/protocol.h"
#include "runtime_mock_sdk.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t clock_us = 1000000;
static bool mock_ready = true, mock_recovery, mock_maintenance, mock_initialized = true;
static bool mock_request_ok = true, connected = true;
static uint16_t mock_protocol = 1;
static unsigned protocol_requests, command_calls, step_calls, sensor_inits, watchdog_calls;
static unsigned gpio_low_calls, gpio_normal_calls, pwm_neutral_calls;
static uint32_t writable = 64;
static uint8_t sent[262144], incoming[8192];
static size_t sent_length, incoming_read, incoming_length;
static void (*rx_callback)(void *);
static void *rx_context;
static void (*core1_entry)(void);
static repeating_timer_callback_t irq_callback;

static void counted_command(control_state_t *state, const float direction[8], float dt,
                            bool stabilization, bool depth_hold) {
    command_calls++;
    control_command(state, direction, dt, stabilization, depth_hold);
}
static void counted_step(control_state_t *state, const control_sample_t *sample, float dt,
                         control_output_t *output) {
    step_calls++;
    control_step(state, sample, dt, output);
}
#define control_command counted_command
#define control_step counted_step
#include "control/runtime.c"
#undef control_command
#undef control_step
#include "usb_comm.h"
#include "usb_rx.h"

uint32_t time_us_32(void) {
    return (uint32_t)clock_us;
}
uint64_t time_us_64(void) {
    return clock_us;
}
absolute_time_t get_absolute_time(void) {
    return clock_us;
}
absolute_time_t from_us_since_boot(uint64_t us) {
    return us;
}
int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return (int64_t)(to - from);
}
void busy_wait_until(absolute_time_t deadline) {
    clock_us = deadline;
}
bool add_repeating_timer_us(int64_t delay, repeating_timer_callback_t callback, void *context,
                            struct repeating_timer *timer) {
    (void)context;
    assert(delay == -1000);
    irq_callback = timer->callback = callback;
    return true;
}
void multicore_launch_core1_with_stack(void (*entry)(void), uint32_t *stack, size_t size) {
    assert(stack != NULL && size >= 4096);
    core1_entry = entry; /* No real thread or uncontrolled infinite loop. */
}
void watchdog_enable(uint32_t delay, bool pause) {
    assert(delay == 1000 && pause);
}
void watchdog_update(void) {
    watchdog_calls++;
}
void tight_loop_contents(void) {
    abort();
}
void gpio_set_outover(uint pin, uint value) {
    assert((pin >= 6 && pin <= 9) || (pin >= 18 && pin <= 21));
    if (value == GPIO_OVERRIDE_LOW)
        gpio_low_calls++;
    else {
        assert(value == GPIO_OVERRIDE_NORMAL);
        gpio_normal_calls++;
    }
}
void pwm_set_gpio_level(uint pin, uint16_t value) {
    assert((pin >= 6 && pin <= 9) || (pin >= 18 && pin <= 21));
    assert(value == 1500);
    pwm_neutral_calls++;
}
uint32_t save_and_disable_interrupts(void) {
    return 0;
}
void restore_interrupts(uint32_t state) {
    assert(state == 0);
}
void __dmb(void) {}
void stdio_set_chars_available_callback(void (*callback)(void *), void *context) {
    rx_callback = callback;
    rx_context = context;
}
uint32_t tud_cdc_available(void) {
    return (uint32_t)(incoming_length - incoming_read);
}
uint32_t tud_cdc_read(void *buffer, uint32_t size) {
    size_t count = incoming_length - incoming_read;
    if (count > size)
        count = size;
    memcpy(buffer, incoming + incoming_read, count);
    incoming_read += count;
    return (uint32_t)count;
}
bool tud_cdc_connected(void) {
    return connected;
}
uint32_t tud_cdc_write_available(void) {
    return writable;
}
uint32_t tud_cdc_write(const void *buffer, uint32_t size) {
    assert(size <= 64 && size <= writable && sent_length + size <= sizeof(sent));
    memcpy(sent + sent_length, buffer, size);
    sent_length += size;
    return size;
}
uint32_t tud_cdc_write_flush(void) {
    return 0;
}
bool bmi270_sensor_init(void) {
    sensor_inits++;
    return true;
}
bool bmi270_sensor_read(float accel[3], float gyro[3], float *temperature) {
    accel[0] = accel[1] = gyro[0] = gyro[1] = gyro[2] = 0;
    accel[2] = -9.80665f;
    *temperature = 25;
    return true;
}
void bmi270_sensor_get_diagnostics(bmi270_sensor_diagnostics_t *out) {
    *out = (bmi270_sensor_diagnostics_t){.initialized = true,
                                         .result = BMI270_SENSOR_OK,
                                         .chip_id = 0x24,
                                         .internal_status = 1,
                                         .samples = 42};
}
void log_info(const char *message) {
    (void)message;
}
void log_warn(const char *message) {
    (void)message;
}
void log_error(const char *message) {
    (void)message;
}
void log_infof(const char *format, ...) {
    (void)format;
}
void log_warnf(const char *format, ...) {
    (void)format;
}
void log_errorf(const char *format, ...) {
    (void)format;
}

static bool request_protocol(uint16_t protocol, uint16_t speed) {
    assert(protocol <= 1 && (speed == 150 || speed == 300 || speed == 600));
    protocol_requests++;
    mock_protocol = protocol;
    return mock_request_ok;
}
static bool protocol_ready(uint16_t protocol, uint16_t speed) {
    (void)speed;
    return mock_ready && protocol == mock_protocol;
}
static bool maintenance_active(void) {
    return mock_maintenance;
}
static bool recovery_required(void) {
    return mock_recovery;
}
static bool outputs_initialized(void) {
    return mock_initialized;
}
static uint16_t output_protocol(void) {
    return mock_protocol;
}
static void flush_tx(void) {
    for (unsigned i = 0; i < 4096; ++i)
        usb_tx_service();
}
static void clear_tx(void) {
    flush_tx();
    sent_length = 0;
}
static int find_ack(uint32_t sequence, uint8_t type, uint32_t *generation) {
    flush_tx();
    int result = -1;
    for (size_t at = 0; at < sent_length;) {
        assert(sent_length - at >= CONTROL_FRAME_OVERHEAD);
        size_t length = CONTROL_FRAME_OVERHEAD + control_get_u16(sent + at + 4);
        control_frame_t frame;
        assert(length <= sent_length - at);
        assert(control_frame_decode(sent + at, length, &frame));
        if (frame.type == FRAME_ACK && frame.sequence == sequence && frame.payload[0] == type) {
            assert(frame.length == 12);
            result = frame.payload[1];
            if (generation)
                *generation = control_get_u32(frame.payload + 4);
        }
        at += length;
    }
    return result;
}
static size_t encode_request(uint8_t *wire, uint32_t session, uint32_t sequence, uint8_t type,
                             const uint8_t *payload, uint16_t length) {
    control_frame_t frame = {
        .type = type, .session = session, .sequence = sequence, .length = length};
    if (length)
        memcpy(frame.payload, payload, length);
    size_t count = control_frame_encode(wire, CONTROL_FRAME_MAX, &frame);
    assert(count != 0);
    return count;
}
#define SESSION UINT32_C(0x12345678)
static void receive(uint32_t sequence, uint8_t type, const uint8_t *payload, uint16_t length) {
    uint8_t wire[CONTROL_FRAME_MAX];
    size_t count = encode_request(wire, SESSION, sequence, type, payload, length);
    control_runtime_receive_at(wire, count, time_us_32());
}
static void init_runtime(void) {
    const control_runtime_hooks_t hooks = {request_protocol,    protocol_ready,
                                           maintenance_active,  recovery_required,
                                           outputs_initialized, output_protocol};
    control_runtime_init(&hooks);
    usb_rx_init();
    assert(core1_entry && irq_callback && rx_callback);
}
static void hello(void) {
    control_runtime_capabilities(7);
    clear_tx();
    receive(1, MSG_HELLO, NULL, 0);
    assert(find_ack(1, MSG_HELLO, NULL) == RESULT_APPLIED);
    clear_tx();
}
static void settings_image(uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE]) {
    memset(wire, 0, CONTROL_SETTINGS_WIRE_SIZE);
    for (size_t i = 0; i < 3; ++i) {
        control_put_f32(wire + 68 + 4 * i, 100);
        control_put_f32(wire + 80 + 4 * i, 1);
    }
    for (size_t i = 0; i < 8; ++i) {
        wire[348 + i] = (uint8_t)i;
        wire[356 + i] = 1;
        control_put_f32(wire + 92 + 4 * (i * 8 + i), 1);
    }
    control_put_u16(wire + 624, mock_protocol);
    control_put_u16(wire + 626, 300);
}
static void begin(uint32_t sequence, uint32_t generation, uint32_t digest) {
    uint8_t payload[12];
    control_put_u32(payload, generation);
    control_put_u32(payload + 4, CONTROL_SETTINGS_WIRE_SIZE);
    control_put_u32(payload + 8, digest);
    receive(sequence, MSG_BEGIN, payload, sizeof(payload));
}
static void chunk(uint32_t sequence, uint32_t offset, const uint8_t *wire, uint16_t count) {
    uint8_t payload[CONTROL_MAX_PAYLOAD];
    control_put_u32(payload, offset);
    memcpy(payload + 4, wire, count);
    receive(sequence, MSG_CHUNK, payload, count + 4);
}
static void commit(uint32_t sequence, uint32_t generation, uint32_t digest) {
    uint8_t payload[8];
    control_put_u32(payload, generation);
    control_put_u32(payload + 4, digest);
    receive(sequence, MSG_COMMIT, payload, sizeof(payload));
}
static void init_core(core1_context_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
    control_init(&ctx->state);
    core1_session(ctx);
    ctx->sensor_initialized = true;
}
static void apply_initial_settings(core1_context_t *ctx) {
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    settings_image(wire);
    uint32_t digest = control_crc32c(wire, sizeof(wire));
    begin(2, 1, digest);
    chunk(3, 0, wire, sizeof(wire));
    clear_tx();
    commit(4, 1, digest);
    assert(pending_commit && active_generation == 0);
    assert(find_ack(4, MSG_COMMIT, NULL) == -1);
    control_runtime_service();
    assert(pending_commit && event_write - event_read == 1);
    init_core(ctx);
    core1_process_events(ctx, time_us_32(), false);
    assert(ctx->generation == 1 && active_generation == 0);
    control_runtime_service();
    assert(active_generation == 1 && active_digest == digest && !pending_commit);
    assert(find_ack(4, MSG_COMMIT, NULL) == RESULT_APPLIED);
    control_runtime_service(); /* Next core0 pass publishes the cleared maintenance gate. */
    clear_tx();
}
static void control(uint32_t sequence, uint32_t flags) {
    uint8_t payload[40] = {0};
    control_put_f32(payload, 0.25f);
    control_put_f32(payload + 32, 1.0f / 60.0f);
    control_put_u32(payload + 36, flags);
    receive(sequence, MSG_CONTROL, payload, sizeof(payload));
}
static void pressure(uint32_t sequence) {
    uint8_t payload[12] = {0};
    control_put_f32(payload, 1);
    control_put_u32(payload + 8, 1);
    receive(sequence, MSG_PRESSURE, payload, sizeof(payload));
}
static void raw(uint32_t sequence, uint16_t thrust) {
    uint8_t payload[16];
    for (size_t i = 0; i < 8; ++i)
        control_put_u16(payload + 2 * i, thrust);
    receive(sequence, MSG_RAW, payload, sizeof(payload));
}
static void assert_neutral(void) {
    uint16_t motors[8];
    control_runtime_get_motors(motors);
    for (size_t i = 0; i < 8; ++i)
        assert(motors[i] == 1000);
}

static void test_staging(void) {
    hello();
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    settings_image(wire);
    uint32_t digest = control_crc32c(wire, sizeof(wire));
    begin(2, 1, digest);
    assert(find_ack(2, MSG_BEGIN, NULL) == RESULT_STAGED);
    chunk(3, 100, wire + 100, 100); /* Gap never stages or activates. */
    assert(find_ack(3, MSG_CHUNK, NULL) == RESULT_INVALID && staging_received == 0);
    chunk(4, 0, wire, 100);
    assert(staging_received == 100);
    chunk(5, 0, wire, 100); /* Duplicate bytes with a fresh sequence are idempotent. */
    assert(find_ack(5, MSG_CHUNK, NULL) == RESULT_STAGED && staging_received == 100);
    wire[0] ^= 1;
    chunk(6, 0, wire, 100);
    assert(find_ack(6, MSG_CHUNK, NULL) == RESULT_INVALID && staging_received == 100);
    wire[0] ^= 1;
    commit(7, 1, digest); /* Missing tail. */
    assert(find_ack(7, MSG_COMMIT, NULL) == RESULT_INVALID && active_generation == 0);
    chunk(8, 100, wire + 100, sizeof(wire) - 100);
    clear_tx();
    mock_ready = false;
    commit(9, 1, digest);
    assert(pending_commit && !authority && protocol_requests == 1);
    control_runtime_service();
    assert(event_write == event_read && active_generation == 0);
    assert(find_ack(9, MSG_COMMIT, NULL) == -1);
    commit(9, 1, digest);
    assert(protocol_requests == 1); /* Lost/delayed ACK retry must not transition twice. */
    mock_ready = true;
    control_runtime_service();
    uint32_t write = event_write;
    control_runtime_service();
    assert(event_write == write && pending_commit && active_generation == 0);
    core1_context_t ctx;
    init_core(&ctx);
    core1_process_events(&ctx, time_us_32(), false);
    assert(active_generation == 0 && ctx.generation == 1);
    control_runtime_service();
    assert(!pending_commit && active_generation == 1);
    assert(find_ack(9, MSG_COMMIT, NULL) == RESULT_APPLIED);
    clear_tx();
    commit(9, 1, digest);
    assert(find_ack(9, MSG_COMMIT, NULL) == RESULT_APPLIED && protocol_requests == 1);
}

static void test_empty_abort_idempotent(void) {
    hello();
    receive(2, MSG_ABORT, NULL, 0);
    assert(find_ack(2, MSG_ABORT, NULL) == RESULT_APPLIED);
    receive(3, MSG_ABORT, NULL, 0);
    assert(find_ack(3, MSG_ABORT, NULL) == RESULT_APPLIED && !staging_active);
    begin(4, 1, 123);
    assert(staging_active);
    receive(5, MSG_ABORT, NULL, 0);
    assert(find_ack(5, MSG_ABORT, NULL) == RESULT_APPLIED && !staging_active);
}

static void test_bad_settings_and_expiry(void) {
    hello();
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    settings_image(wire);
    uint32_t digest = control_crc32c(wire, sizeof(wire));
    begin(2, 1, digest ^ 1u);
    chunk(3, 0, wire, sizeof(wire));
    commit(4, 1, digest ^ 1u);
    assert(find_ack(4, MSG_COMMIT, NULL) == RESULT_INVALID && protocol_requests == 0);
    control_put_f32(wire, NAN);
    digest = control_crc32c(wire, sizeof(wire));
    begin(5, 1, digest);
    chunk(6, 0, wire, sizeof(wire));
    commit(7, 1, digest);
    assert(find_ack(7, MSG_COMMIT, NULL) == RESULT_INVALID && active_generation == 0);
    begin(8, 2, digest);
    clock_us += SETTINGS_TIMEOUT_US + 1;
    chunk(9, 0, wire, sizeof(wire));
    assert(find_ack(9, MSG_CHUNK, NULL) == RESULT_NOT_READY && !staging_active);
}

static void test_queued_commit_timeout(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    uint8_t wire[CONTROL_SETTINGS_WIRE_SIZE];
    settings_image(wire);
    control_put_u16(wire + 624, 0); /* Change physical output protocol. */
    uint32_t digest = control_crc32c(wire, sizeof(wire));
    begin(5, 2, digest);
    chunk(6, 0, wire, sizeof(wire));
    commit(7, 2, digest);
    control_runtime_service();
    assert(pending_commit && event_write != event_read);
    clock_us += 7000001;
    control_runtime_service();
    assert(!pending_commit && pending_sequence == 0);
    assert(find_ack(7, MSG_COMMIT, NULL) == RESULT_NOT_READY);
    core1_process_events(&ctx, time_us_32(), false);
    assert(ctx.generation == 1 && active_generation == 1); /* Late queue cannot apply. */
    control(8, 4);
    assert(find_ack(8, MSG_CONTROL, NULL) == RESULT_NOT_READY);
    assert_neutral();
    mock_recovery = true;
    receive(9, MSG_QUERY, NULL, 0);
    uint32_t generation = 0;
    assert(find_ack(9, MSG_QUERY, &generation) == RESULT_APPLIED && generation == 1);
}

static void test_duplicate_fingerprint(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    control(5, 4);
    uint32_t write = event_write;
    control(5, 4);
    assert(event_write == write);
    control(5, 5); /* Valid frame CRC, changed payload on same sequence. */
    assert(find_ack(5, MSG_CONTROL, NULL) == RESULT_INVALID && event_write == write);
    clear_tx();
    control(5, 4);
    assert(find_ack(5, MSG_CONTROL, NULL) == -1 && event_write == write);
    control(3, 4);
    assert(find_ack(3, MSG_CONTROL, NULL) == RESULT_STALE && event_write == write);
    begin(6, 2, 123);
    assert(find_ack(6, MSG_BEGIN, NULL) == RESULT_STAGED);
    begin(6, 3, 456);
    assert(find_ack(6, MSG_BEGIN, NULL) == RESULT_INVALID && staging_generation == 2);
    clear_tx();
    begin(6, 2, 123);
    assert(find_ack(6, MSG_BEGIN, NULL) == RESULT_STAGED && staging_generation == 2);
}

static void test_gate_retry(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    raw(5, 1000);
    receive(6, 0x25, NULL, 0);
    assert(pending_gate && maintenance_latched && active_session == SESSION);
    for (unsigned round = 0; round < 3; ++round) {
        assert_neutral();
        control_runtime_output_serviced();
        control_runtime_service();
        assert(pending_gate && find_ack(6, 0x25, NULL) == -1);
    }
    /* Fill the actual bounded urgent TX queue; the first gate ACK is dropped. */
    writable = 0;
    uint8_t filler[CONTROL_FRAME_MAX];
    size_t count = encode_request(filler, SESSION, 99, 0xee, NULL, 0);
    for (unsigned i = 0; i < 20; ++i)
        (void)usb_tx_packet(filler, count, true);
    uint32_t drops = usb_tx_dropped();
    assert_neutral();
    control_runtime_output_serviced();
    control_runtime_service();
    assert(!pending_gate && active_session == 0 && maintenance_latched);
    assert(usb_tx_dropped() > drops);
    writable = 64;
    clear_tx();
    receive(6, 0x25, NULL, 0);
    uint32_t generation = 0;
    assert(find_ack(6, 0x25, &generation) == RESULT_APPLIED && generation == 1);
    uint8_t wire[CONTROL_FRAME_MAX];
    count = encode_request(wire, SESSION + 1, 1, MSG_HELLO, NULL, 0);
    control_runtime_receive(wire, count);
    assert(active_session == SESSION + 1);
    clear_tx();
    receive(6, 0x25, NULL, 0);
    assert(find_ack(6, 0x25, NULL) == RESULT_NOT_READY);
}

static void test_irq_starvation(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    raw(5, 1200);
    uint16_t motors[8];
    control_runtime_get_motors(motors);
    assert(motors[0] == 1200 && safety_armed);
    clock_us += OUTPUT_TIMEOUT_US + 1;
    assert(irq_callback(&safety_timer)); /* No core0 service or USB progress. */
    assert(safety_latched && gpio_low_calls == 8);
    control_runtime_service();
    assert_neutral();
    assert(active_session == 0 && maintenance_latched);
    control(6, 4);
    assert(find_ack(6, MSG_CONTROL, NULL) == RESULT_NOT_READY);
    receive(1, MSG_HELLO, NULL, 0);
    assert(find_ack(1, MSG_HELLO, NULL) == RESULT_NOT_READY);
    control_runtime_output_serviced();
    assert(gpio_normal_calls == 8);
    uint8_t wire[CONTROL_FRAME_MAX];
    size_t count = encode_request(wire, SESSION + 1, 1, MSG_HELLO, NULL, 0);
    control_runtime_receive(wire, count);
    assert(active_session == SESSION + 1 && active_generation == 0);
}

static void test_pwm_and_host_irq(void) {
    mock_protocol = 0;
    uint16_t motors[8];
    for (size_t i = 0; i < 8; ++i)
        motors[i] = 1200;
    control_runtime_legacy_input(motors);
    control_runtime_get_motors(motors);
    assert(safety_armed);
    clock_us += HOST_TIMEOUT_US + 1;
    core0_heartbeat = safety_output_us = time_us_32(); /* Outputs continue, host does not. */
    assert(irq_callback(&safety_timer));
    assert(pwm_neutral_calls == 8 && gpio_low_calls == 0 && safety_latched);
}

static void test_commands_pressure_and_sensor(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    control(5, 4);
    control(6, 4);
    core1_process_events(&ctx, time_us_32(), false);
    assert(command_calls == 0 && event_write - event_read == 2);
    core1_process_events(&ctx, time_us_32(), true);
    assert(command_calls == 1 && event_write - event_read == 1);
    control_sample_t sample = {.accel = {0, 0, -9.80665f}};
    core1_sample(&ctx, &sample, 25, time_us_32());
    assert(step_calls == 1);
    core1_process_events(&ctx, time_us_32(), true);
    assert(command_calls == 2 && event_write == event_read);
    control(7, 6); /* Missing pressure: must not advance desired depth or PID. */
    control_state_t before = ctx.state;
    core1_process_events(&ctx, time_us_32(), true);
    assert(command_calls == 2 && !ctx.command_valid);
    assert(memcmp(&before, &ctx.state, sizeof(before)) == 0);
    pressure(8);
    core1_process_events(&ctx, time_us_32(), true);
    assert(command_calls == 2 && !ctx.command_valid);
    control(9, 6);
    core1_process_events(&ctx, time_us_32(), true);
    assert(command_calls == 3 && ctx.command_valid && ctx.stats.depth == 1);
    uint32_t host_time = ctx.command_time;
    clock_us += HOST_TIMEOUT_US + 1;
    pressure(10);
    core1_process_events(&ctx, time_us_32(), true);
    assert(ctx.command_time == host_time && !core1_control_permitted(&ctx, time_us_32()));
    core1_snapshot(&ctx, time_us_32());
    assert((ctx.snapshot.health & 4u) == 0);
    assert(ctx.snapshot.sensor.chip_id == 0x24 && ctx.snapshot.sensor.samples == 42);
}

static void test_stale_hol_commit(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    const uint32_t flags[] = {4, 0, 4};
    for (size_t i = 0; i < 3; ++i) {
        control_event_t event = {.type = MSG_CONTROL,
                                 .session = SESSION,
                                 .sequence = 20,
                                 .received_us = time_us_32(),
                                 .flags = flags[i]};
        if (i == 0)
            event.received_us -= HOST_TIMEOUT_US + 1;
        if (i == 2)
            accepted_floor = 20;
        assert(event_push(&event));
        event = (control_event_t){.type = MSG_COMMIT,
                                  .session = SESSION,
                                  .sequence = 30,
                                  .generation = 2,
                                  .digest = 123,
                                  .settings = ctx.state.settings};
        commit_authorized_sequence = 30;
        assert(event_push(&event));
        core1_process_events(&ctx, time_us_32(), false);
        assert(event_read == event_write && ctx.generation == 2 && command_calls == 0);
    }
}

static void test_sensor_retry_handshake(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    ctx.sensor_initialized = false;
    clock_us += 1000001;
    raw(5, 1000);
    assert_neutral();
    assert(control_runtime_output_permitted());
    core1_retry_sensor(&ctx, time_us_32());
    assert(!sensor_retry_requested &&
           sensor_inits == 0); /* Neutral raw calibration is permitted. */
    control_runtime_inhibit();
    assert_neutral();
    core1_retry_sensor(&ctx, time_us_32());
    assert(sensor_retry_requested && !sensor_retry_inhibited && sensor_inits == 0);
    control_runtime_service();
    assert_neutral();
    control_runtime_output_serviced();
    assert(sensor_retry_inhibited);
    core1_retry_sensor(&ctx, time_us_32());
    assert(sensor_inits == 1 && !sensor_retry_requested && !sensor_retry_inhibited);
}

static uint8_t extended_buffer[CONTROL_FRAME_MAX], legacy_buffer[18];
static usb_packet_reader_t readers[] = {
    {USB_CONTROL_START_BYTE, extended_buffer, CONTROL_FRAME_MAX, 0, USB_PACKET_CONTROL, 0},
    {USB_INPUT_START_BYTE, legacy_buffer, sizeof(legacy_buffer), 0, USB_PACKET_COMMAND, 0}};
static void legacy_packet(uint8_t *wire, uint16_t thrust) {
    wire[0] = USB_INPUT_START_BYTE;
    for (size_t i = 0; i < 8; ++i)
        control_put_u16(wire + 1 + 2 * i, thrust);
    wire[17] = usb_calculate_checksum(wire, 17);
}
static unsigned parser_legacy_packets;
static void dispatch(usb_packet_kind_t kind) {
    if (kind == USB_PACKET_CONTROL) {
        control_runtime_receive_at(extended_buffer, readers[0].packet_size,
                                   (uint32_t)readers[0].last_byte_time);
    } else if (kind == USB_PACKET_INVALID) {
        control_runtime_inhibit();
    } else if (kind == USB_PACKET_COMMAND) {
        uint16_t motors[8];
        absolute_time_t received;
        assert(usb_parse_packet(legacy_buffer, sizeof(legacy_buffer), motors, 8, &received));
        parser_legacy_packets++;
        control_runtime_legacy_input_at(motors, (uint32_t)readers[1].last_byte_time);
    }
}
static void feed(const uint8_t *wire, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        dispatch(usb_process_byte(readers, 2, wire[i], clock_us));
    }
}
static void test_parser_atomic_and_timeout_tail(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    uint8_t legacy[18], wire[CONTROL_FRAME_MAX];
    legacy_packet(legacy, 1600);
    size_t count = encode_request(wire, SESSION, 5, 0xfe, legacy, sizeof(legacy));
    wire[count - 1] ^= 1;
    feed(wire, count);
    assert(parser_legacy_packets == 0 && !authority);
    assert_neutral();
    count = encode_request(wire, SESSION, 6, 0xfe, legacy, sizeof(legacy));
    feed(wire, 14); /* Truncated frame then a payload shaped as a legacy packet after timeout. */
    clock_us += 100001;
    feed(legacy, sizeof(legacy));
    assert(parser_legacy_packets == 1); /* Parser resyncs; negotiated authority gate refuses it. */
    assert_neutral();
    assert(!authority && active_session == SESSION);
    wire[1] = 2;
    feed(wire, 6);
    feed(legacy, sizeof(legacy));
    assert_neutral();
}
static void irq_receive(const uint8_t *wire, size_t length) {
    assert(length <= sizeof(incoming));
    memcpy(incoming, wire, length);
    incoming_read = 0;
    incoming_length = length;
    while (incoming_read != incoming_length)
        rx_callback(rx_context);
}
static void test_rx_arrival_lease_and_overflow(void) {
    uint8_t wire[18];
    legacy_packet(wire, 1500);
    irq_receive(wire, sizeof(wire));
    uint32_t arrival = time_us_32();
    clock_us += HOST_TIMEOUT_US + 1;
    usb_packet_kind_t kind = usb_poll(readers, 2);
    assert(kind == USB_PACKET_COMMAND && readers[1].last_byte_time == arrival);
    dispatch(kind);
    assert(raw_received_us == arrival);
    assert_neutral(); /* Buffered bytes cannot acquire a new dequeue-time lease. */
    uint8_t large[4096];
    memset(large, 0x5a, sizeof(large));
    irq_receive(large, sizeof(large));
    readers[0].index = 3;
    assert(usb_poll(readers, 2) == USB_PACKET_INVALID);
    assert(readers[0].index == 0 && readers[1].index == 0);
    absolute_time_t ignored;
    assert(usb_rx_get(&ignored) == PICO_ERROR_TIMEOUT);
}
static void test_rx_old_extended_lease(void) {
    hello();
    core1_context_t ctx;
    apply_initial_settings(&ctx);
    uint8_t payload[16], wire[CONTROL_FRAME_MAX];
    for (size_t i = 0; i < 8; ++i)
        control_put_u16(payload + 2 * i, 1500);
    size_t count = encode_request(wire, SESSION, 5, MSG_RAW, payload, sizeof(payload));
    irq_receive(wire, count);
    clock_us += HOST_TIMEOUT_US + 1;
    dispatch(usb_poll(readers, 2));
    assert_neutral();
    assert(!control_runtime_output_permitted());
}

int main(int argc, char **argv) {
    assert(argc == 2);
    init_runtime();
#define CASE(name)                                                                                 \
    if (strcmp(argv[1], #name) == 0) {                                                             \
        test_##name();                                                                             \
        puts("PASS " #name);                                                                       \
        return 0;                                                                                  \
    }
    CASE(staging)
    CASE(empty_abort_idempotent)
    CASE(bad_settings_and_expiry)
    CASE(queued_commit_timeout)
    CASE(duplicate_fingerprint)
    CASE(gate_retry)
    CASE(irq_starvation)
    CASE(pwm_and_host_irq)
    CASE(commands_pressure_and_sensor)
    CASE(stale_hol_commit)
    CASE(sensor_retry_handshake)
    CASE(parser_atomic_and_timeout_tail)
    CASE(rx_arrival_lease_and_overflow)
    CASE(rx_old_extended_lease)
#undef CASE
    fprintf(stderr, "Unknown scenario: %s\n", argv[1]);
    return 2;
}
