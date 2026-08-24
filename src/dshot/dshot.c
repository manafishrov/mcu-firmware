/*
 * DShot protocol implementation for RP2040 PIO with bidirectional telemetry.
 * TX derived from pio-dshot by Simon Wunderlich (MIT License).
 * RX uses Betaflight-style oversampled edge detection with run-length decoding.
 *
 * EDT handling follows Betaflight's dshot.c:
 * https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot.c
 *
 * Oversampled telemetry decoding adapted from Betaflight's dshot_bidir_pico.c:
 * https://github.com/betaflight/betaflight/blob/master/src/platform/PICO/dshot_bidir_pico.c
 */

#include "dshot.h"
#include "dshot.pio.h"
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/structs/clocks.h>
#include <hardware/structs/io_bank0.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static bool dshot_pio_prog_loaded[2] = {false, false};
static uint dshot_pio_prog_offset[2] = {0, 0};

static uint pio_index(PIO pio) {
    return pio == pio0 ? 0 : 1;
}

/*
 * GCR (5-bit) to nibble (4-bit) decoding table.
 * Bidirectional DShot telemetry uses GCR encoding: 4 nibbles encoded as
 * 4 x 5-bit GCR symbols = 20 bits transmitted. 0xFF marks invalid symbols.
 */
static const uint8_t gcr_table[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07, 0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
};

/*
 * EDT type lookup table, indexed by telemetry_type >> 1.
 * Matches Betaflight's extendedTelemetryLookup[]. The telemetry_type field
 * is 4 bits (bits 11:8 of the 12-bit value). Dropping the tag bit (bit 0)
 * gives a 3-bit index into this table.
 *
 * EDT prefix → index:
 *   0x00 >> 1 = 0  eRPM (handled by conditional, never looked up)
 *   0x02 >> 1 = 1  Temperature (degrees C)
 *   0x04 >> 1 = 2  Voltage (0.25V per LSB, range 0-63.75V)
 *   0x06 >> 1 = 3  Current (1A per LSB, range 0-255A)
 *   0x08 >> 1 = 4  Debug 1
 *   0x0A >> 1 = 5  Debug 2
 *   0x0C >> 1 = 6  Debug 3 (stress level in EDT v2.0.0+)
 *   0x0E >> 1 = 7  State / Events
 */
static const enum dshot_telemetry_type edt_type_lookup[8] = {
    DSHOT_TELEMETRY_TYPE_ERPM,    DSHOT_TELEMETRY_TYPE_TEMPERATURE,  DSHOT_TELEMETRY_TYPE_VOLTAGE,
    DSHOT_TELEMETRY_TYPE_CURRENT, DSHOT_TELEMETRY_TYPE_DEBUG1,       DSHOT_TELEMETRY_TYPE_DEBUG2,
    DSHOT_TELEMETRY_TYPE_DEBUG3,  DSHOT_TELEMETRY_TYPE_STATE_EVENTS,
};

/* ---- Oversampled telemetry decoder (Betaflight-derived) ---- */

#define OVERSAMPLE_WORDS 4
#define OVERSAMPLE_COMPLETION_WORDS 1
#define OVERSAMPLE_TOTAL_WORDS (OVERSAMPLE_WORDS + OVERSAMPLE_COMPLETION_WORDS)
#define MAX_EDGES 24
#define PIO_CYCLES_PER_TX_BIT 125
#define PIO_CYCLES_PER_SAMPLE 18
#define RX_BIT_RATIO_NUM 5
#define RX_BIT_RATIO_DEN 4

enum decode_result {
    DECODE_OK = 0,
    DECODE_FAIL_EDGE_COUNT,
    DECODE_FAIL_BIT_COUNT,
    DECODE_FAIL_GCR,
    DECODE_FAIL_CRC,
};

/*
 * Samples per RX bit = (PIO_CYCLES_PER_TX_BIT * RX_BIT_RATIO_DEN)
 *                      / (RX_BIT_RATIO_NUM * PIO_CYCLES_PER_SAMPLE)
 *                    = (125 * 4) / (5 * 18) = 500 / 90 = 5.556...
 * Default bits_per_sample = 1 / 5.556 = 0.18
 */
#define DEFAULT_BITS_PER_SAMPLE 0.18f

#define CALIBRATION_FRAMES 32
#define ZERO_RPM_EDGES 15
#define ZERO_RPM_BIT_SPAN 18
#define ZERO_RPM_VALUE 0xFFF0

static uint8_t length_transitions[4];
static bool calibration_complete;
static uint32_t calibration_total_span;
static int calibration_frame_count;

static void set_length_transitions(float bits_per_sample, bool strict);

static void ensure_decoder_initialized(void) {
    static bool initialized;

    if (!initialized) {
        set_length_transitions(DEFAULT_BITS_PER_SAMPLE, false);
        initialized = true;
    }
}

static int collect_edge_diffs(const uint32_t *buffer, uint8_t *edge_diffs) {
    bool ones = buffer[0] >> 31;
    uint32_t w0 = buffer[0];
    uint32_t w1 = buffer[1];
    uint32_t w2 = buffer[2];
    uint32_t w3 = buffer[3];
    int shift_remaining = 128;
    int edge_count = 0;

    while (shift_remaining > 0 && edge_count < MAX_EDGES) {
        uint32_t test = ones ? ~w0 : w0;
        if (test == 0) {
            break;
        }

        int run = __builtin_clz(test);
        int complement = 32 - run;

        edge_diffs[edge_count++] = run;
        w0 = (w0 << run) | (w1 >> complement);
        w1 = (w1 << run) | (w2 >> complement);
        w2 = (w2 << run) | (w3 >> complement);
        w3 = w3 << run;
        ones = !ones;
        shift_remaining -= run;
    }

    if (!ones && edge_count > 0) {
        edge_count--;
    }

    return edge_count;
}

static int decode_run_length(uint8_t diff) {
    if (diff < length_transitions[1]) {
        return 1;
    }
    if (diff < length_transitions[2]) {
        return 2;
    }
    if (diff < length_transitions[3]) {
        return 3;
    }
    return 0;
}

static enum decode_result build_gcr_word(const uint8_t *edge_diffs, int edge_count,
                                         uint32_t *gcr20_out) {
    if (edge_count < 2 || edge_count > 21) {
        return DECODE_FAIL_EDGE_COUNT;
    }

    uint32_t core_gcr = 0;
    uint32_t core_bits = 0;

    for (int i = 0; i < edge_count; ++i) {
        int len = decode_run_length(edge_diffs[i]);
        if (len == 0) {
            return DECODE_FAIL_GCR;
        }

        core_gcr <<= len;
        core_gcr |= 1U << (len - 1U);
        core_bits += len;
        if (core_bits >= 21U) {
            break;
        }
    }

    int32_t padding = 21 - core_bits;
    if (padding < 0) {
        return DECODE_FAIL_BIT_COUNT;
    }

    *gcr20_out = core_gcr << padding;
    if (padding > 0) {
        *gcr20_out |= 1U << (padding - 1);
    }

    return DECODE_OK;
}

static enum decode_result decode_gcr_word(uint32_t gcr20, uint32_t *out_value) {
    uint8_t n3 = gcr_table[(gcr20 >> 15) & 0x1F];
    uint8_t n2 = gcr_table[(gcr20 >> 10) & 0x1F];
    uint8_t n1 = gcr_table[(gcr20 >> 5) & 0x1F];
    uint8_t n0 = gcr_table[gcr20 & 0x1F];

    if ((n0 | n1 | n2 | n3) & 0xF0) {
        return DECODE_FAIL_GCR;
    }

    uint32_t value = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0;
    uint32_t csum = value ^ (value >> 8);

    csum ^= csum >> 4;
    if ((csum & 0xF) != 0xF) {
        return DECODE_FAIL_CRC;
    }

    *out_value = value;
    return DECODE_OK;
}

static void update_zero_rpm_calibration(const uint8_t *edge_diffs, int edge_count, uint32_t value) {
    if (calibration_complete || value != ZERO_RPM_VALUE || edge_count != ZERO_RPM_EDGES) {
        return;
    }

    uint32_t span = 0;
    for (int i = 1; i < ZERO_RPM_EDGES; ++i) {
        span += edge_diffs[i];
    }

    if (span >= 70 && span <= 125) {
        calibration_total_span += span;
        calibration_frame_count++;
    }

    if (calibration_frame_count == CALIBRATION_FRAMES) {
        float avg_span = (float)calibration_total_span / CALIBRATION_FRAMES;
        float samples_per_bit = avg_span / ZERO_RPM_BIT_SPAN;

        set_length_transitions(1.0f / samples_per_bit, true);
        calibration_complete = true;
    }
}

void dshot_controller_reset_calibration(void) {
    calibration_complete = false;
    calibration_total_span = 0;
    calibration_frame_count = 0;
    set_length_transitions(DEFAULT_BITS_PER_SAMPLE, false);
}

static void set_length_transitions(float bits_per_sample, bool strict) {
    int length = 0;
    float bits = 0.5f;
    int samples = 0;
    while (length < 4) {
        bits += bits_per_sample;
        samples++;
        if ((int)bits >= length + 1) {
            length_transitions[length] = samples;
            length++;
        }
    }
    if (!strict) {
        length_transitions[3] += 2;
    }
}

/*
 * Decode 4 words of oversampled telemetry into a 16-bit GCR value.
 * Algorithm (from Betaflight):
 *   1. Scan 128-bit sample stream for edge transitions using __builtin_clz
 *   2. Record run lengths between edges
 *   3. Convert run lengths to GCR bits via transition table
 *   4. Reconstruct 21-bit GCR pattern (20 data + 1 start)
 *   5. Decode 4 x 5-bit GCR symbols to nibbles, verify checksum
 * Returns 16-bit value (12-bit data + 4-bit CRC) or DSHOT_TELEMETRY_INVALID.
 */
static enum decode_result decode_oversampled_telemetry(const uint32_t *buffer,
                                                       uint32_t *out_value) {
    uint32_t gcr20;
    uint8_t edge_diffs[MAX_EDGES];
    enum decode_result result;

    ensure_decoder_initialized();

    int edge_count = collect_edge_diffs(buffer, edge_diffs);
    result = build_gcr_word(edge_diffs, edge_count, &gcr20);
    if (result != DECODE_OK) {
        return result;
    }

    result = decode_gcr_word(gcr20, out_value);
    if (result != DECODE_OK) {
        return result;
    }

    update_zero_rpm_calibration(edge_diffs, edge_count, *out_value);
    return DECODE_OK;
}

/* ---- End oversampled decoder ---- */

static void dshot_sm_config_set_pin(struct dshot_controller *controller, int pin) {
    sm_config_set_out_pins(&controller->c, pin, 1);
    sm_config_set_set_pins(&controller->c, pin, 1);
    sm_config_set_in_pins(&controller->c, pin);
    sm_config_set_jmp_pin(&controller->c, pin);

    pio_gpio_init(controller->pio, pin);
    gpio_set_pulls(pin, true, false);
}

void dshot_controller_init(struct dshot_controller *controller, uint16_t dshot_speed, PIO pio,
                           uint8_t sm, int pin, int channels) {
    memset(controller, 0, sizeof(*controller));
    controller->pio = pio;
    controller->sm = sm;
    controller->num_channels = channels;
    controller->speed = dshot_speed;
    controller->pin = pin;
    controller->command_last_time = get_absolute_time();

    for (int i = 0; i < controller->num_channels; i++) {
        controller->motor[i].last_throttle_value = UINT16_MAX;
        dshot_throttle(controller, i, 0);
    }

    uint pi = pio_index(pio);
    if (!dshot_pio_prog_loaded[pi]) {
        dshot_pio_prog_offset[pi] = pio_add_program(pio, &pio_dshot_program);
        dshot_pio_prog_loaded[pi] = true;
    }

    controller->c = pio_dshot_program_get_default_config(dshot_pio_prog_offset[pi]);

    sm_config_set_out_shift(&controller->c, false, false, 32);
    sm_config_set_in_shift(&controller->c, false, true, 32);

    dshot_sm_config_set_pin(controller, pin);

    float clkdiv = (float)clock_get_hz(clk_sys) / (1000.0F * (float)dshot_speed * 125.0F);
    sm_config_set_clkdiv(&controller->c, clkdiv);

    pio_sm_init(pio, sm, dshot_pio_prog_offset[pi], &controller->c);
    pio_sm_set_enabled(pio, sm, true);
}

void dshot_controller_deinit(struct dshot_controller *controller) {
    if (controller->num_channels == 0) {
        memset(controller, 0, sizeof(*controller));
        return;
    }

    pio_sm_set_enabled(controller->pio, controller->sm, false);
    pio_sm_clear_fifos(controller->pio, controller->sm);
    pio_sm_restart(controller->pio, controller->sm);

    for (uint8_t i = 0; i < controller->num_channels; ++i) {
        uint gpio = controller->pin + i;
        gpio_set_function(gpio, GPIO_FUNC_SIO);
        gpio_disable_pulls(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
        gpio_put(gpio, 0);
    }

    memset(controller, 0, sizeof(*controller));
}

void dshot_register_telemetry_cb(struct dshot_controller *controller,
                                 dshot_telemetry_callback_t telemetry_cb, void *context) {
    controller->telemetry_cb = telemetry_cb;
    controller->telemetry_cb_context = context;
}

/*
 * Decode eRPM from the 12-bit telemetry value (format: eeem mmmm mmmm).
 * Returns eRPM / 100 (each LSB = 100 eRPM), 0 for motor stopped,
 * or DSHOT_TELEMETRY_INVALID if the period is zero.
 */
static uint32_t dshot_decode_erpm_telemetry_value(uint16_t value) {
    if (value == 0x0FFF) {
        return 0;
    }

    uint16_t period = (value & 0x01FF) << ((value & 0xFE00) >> 9);
    if (period == 0) {
        return DSHOT_TELEMETRY_INVALID;
    }

    return (600000 + period / 2) / period;
}

/*
 * Decode telemetry value into type + decoded value.
 * EDT detection starts only after the ESC acknowledges command 13. Before
 * that handshake, even-nibble eRPM frames must not be mistaken for EDT data.
 *
 * EDT frame discrimination (4-bit type field, bits 11:8):
 *   - Bit 0 = 1 (odd) -> always eRPM
 *   - Type = 0x00     -> always eRPM
 *   - Bit 0 = 0, type != 0 -> EDT frame, type >> 1 indexes edt_type_lookup
 */
static void dshot_decode_telemetry_value(struct dshot_controller *controller, uint16_t raw_value,
                                         uint32_t *decoded, enum dshot_telemetry_type *type) {
    struct dshot_motor *motor = &controller->motor[controller->channel];

    unsigned telemetry_type = (raw_value & 0x0F00) >> 8;
    bool is_erpm = !motor->edt_enabled || (telemetry_type & 0x01) || (telemetry_type == 0);

    if (is_erpm) {
        *decoded = dshot_decode_erpm_telemetry_value(raw_value);
        *type = DSHOT_TELEMETRY_TYPE_ERPM;
    } else {
        unsigned type_index = telemetry_type >> 1;
        *type = edt_type_lookup[type_index];
        *decoded = raw_value & 0x00FF;
    }
}

static bool dshot_handle_edt_handshake(struct dshot_motor *motor, uint16_t raw_value) {
    if ((raw_value & 0x0F00) != 0x0E00 ||
        motor->current_command != DSHOT_EXTENDED_TELEMETRY_ENABLE) {
        return false;
    }

    motor->edt_enabled = true;
    return true;
}

static void dshot_update_telemetry_data(struct dshot_motor *motor, enum dshot_telemetry_type type,
                                        uint32_t value, uint32_t now_ms) {
    motor->telemetry_data[type] = value;
    motor->telemetry_types |= (1 << type);

    if (type == DSHOT_TELEMETRY_TYPE_ERPM) {
        motor->last_erpm_ms = now_ms;
    }

    if (type == DSHOT_TELEMETRY_TYPE_TEMPERATURE && value > motor->max_temp) {
        motor->max_temp = value;
    }
}

/*
 * Update windowed telemetry quality statistics.
 * Uses a rotating bucket array to maintain a sliding 600ms window.
 */
static void dshot_update_telemetry_quality(struct dshot_telemetry_quality *quality,
                                           bool packet_valid, uint32_t current_ms) {
    uint8_t bucket_index =
        (current_ms / DSHOT_TELEMETRY_QUALITY_BUCKET_MS) % DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT;

    if (bucket_index != quality->last_bucket_index) {
        quality->packet_count_sum -= quality->packet_count_array[bucket_index];
        quality->invalid_count_sum -= quality->invalid_count_array[bucket_index];
        quality->packet_count_array[bucket_index] = 0;
        quality->invalid_count_array[bucket_index] = 0;
        quality->last_bucket_index = bucket_index;
    }

    quality->packet_count_sum++;
    quality->packet_count_array[bucket_index]++;
    if (!packet_valid) {
        quality->invalid_count_sum++;
        quality->invalid_count_array[bucket_index]++;
    }
}

/*
 * Process oversampled telemetry received from PIO.
 * Decodes 4 words of oversampled data via edge detection → run-length → GCR,
 * then extracts telemetry type/value and updates motor state.
 */
static void dshot_receive_oversampled(struct dshot_controller *controller, const uint32_t *buffer) {
    struct dshot_motor *motor = &controller->motor[controller->channel];
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    if (buffer[0] == 0 && buffer[1] == 0 && buffer[2] == 0 && buffer[3] == 0) {
        motor->stats.rx_timeout++;
        dshot_update_telemetry_quality(&motor->quality, false, now_ms);
        return;
    }

    uint32_t frame;
    enum decode_result result = decode_oversampled_telemetry(buffer, &frame);
    if (result != DECODE_OK) {
        switch (result) {
        case DECODE_FAIL_EDGE_COUNT:
        case DECODE_FAIL_BIT_COUNT:
        case DECODE_FAIL_GCR:
            motor->stats.rx_bad_gcr++;
            break;
        case DECODE_FAIL_CRC:
            motor->stats.rx_bad_crc++;
            break;
        default:
            motor->stats.rx_bad_crc++;
            break;
        }
        dshot_update_telemetry_quality(&motor->quality, false, now_ms);
        return;
    }

    uint16_t raw_value = (frame >> 4) & 0x0FFF;

    if (dshot_handle_edt_handshake(motor, raw_value)) {
        motor->stats.rx_frames++;
        dshot_update_telemetry_quality(&motor->quality, true, now_ms);
        return;
    }

    enum dshot_telemetry_type type;
    uint32_t decoded;
    dshot_decode_telemetry_value(controller, raw_value, &decoded, &type);

    if (decoded == DSHOT_TELEMETRY_INVALID) {
        motor->stats.rx_bad_type++;
        dshot_update_telemetry_quality(&motor->quality, false, now_ms);
        return;
    }

    motor->stats.rx_frames++;
    dshot_update_telemetry_data(motor, type, decoded, now_ms);
    dshot_update_telemetry_quality(&motor->quality, true, now_ms);

    if (controller->telemetry_cb) {
        controller->telemetry_cb(controller->telemetry_cb_context, controller->channel, type,
                                 decoded);
    }
}

static void dshot_cycle_channel(struct dshot_controller *controller) {
    pio_sm_set_enabled(controller->pio, controller->sm, false);

    controller->channel = (controller->channel + 1) % controller->num_channels;
    dshot_sm_config_set_pin(controller, controller->pin + controller->channel);
    pio_sm_init(controller->pio, controller->sm, dshot_pio_prog_offset[pio_index(controller->pio)],
                &controller->c);

    pio_sm_set_enabled(controller->pio, controller->sm, true);
}

void dshot_loop_async_start(struct dshot_controller *controller) {
    if (controller->num_channels > 1) {
        dshot_cycle_channel(controller);
    }

    struct dshot_motor *motor = &controller->motor[controller->channel];
    if (pio_sm_is_tx_fifo_empty(controller->pio, controller->sm)) {
        motor->stats.tx_frames++;
        pio_sm_put(controller->pio, controller->sm, ~(uint32_t)motor->frame << 16);
        uint32_t cycles = (25 * controller->speed * 125) / 1000;
        pio_sm_put(controller->pio, controller->sm, cycles);
    }
}

#define RX_READ_TIMEOUT_US 500

static bool dshot_read_rx_words(struct dshot_controller *controller, uint32_t *buffer) {
    absolute_time_t deadline = make_timeout_time_us(RX_READ_TIMEOUT_US);
    for (int i = 0; i < OVERSAMPLE_TOTAL_WORDS; i++) {
        while (pio_sm_is_rx_fifo_empty(controller->pio, controller->sm)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                while (!pio_sm_is_rx_fifo_empty(controller->pio, controller->sm)) {
                    (void)pio_sm_get(controller->pio, controller->sm);
                }
                return false;
            }
        }
        uint32_t word = pio_sm_get(controller->pio, controller->sm);
        if (i < OVERSAMPLE_WORDS) {
            buffer[i] = word;
        }
    }
    return true;
}

void dshot_loop_async_complete(struct dshot_controller *controller) {
    uint32_t buffer[OVERSAMPLE_WORDS] = {0};
    bool ok = dshot_read_rx_words(controller, buffer);

    if (!ok) {
        pio_sm_set_enabled(controller->pio, controller->sm, false);
        pio_sm_init(controller->pio, controller->sm,
                    dshot_pio_prog_offset[pio_index(controller->pio)], &controller->c);
        pio_sm_set_enabled(controller->pio, controller->sm, true);

        struct dshot_motor *motor = &controller->motor[controller->channel];
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        motor->stats.rx_timeout++;
        dshot_update_telemetry_quality(&motor->quality, false, now_ms);
    } else {
        dshot_receive_oversampled(controller, buffer);
    }

    struct dshot_motor *motor = &controller->motor[controller->channel];
    if (motor->command_counter > 0) {
        motor->command_counter--;
        if (motor->command_counter == 0) {
            motor->frame = motor->last_throttle_frame;
            motor->current_command = 0;
        }
    }

    if (absolute_time_diff_us(controller->command_last_time, get_absolute_time()) >
        DSHOT_IDLE_THRESHOLD) {
        for (int i = 0; i < controller->num_channels; i++) {
            dshot_throttle(controller, i, 0);
        }
    }
}

void dshot_loop(struct dshot_controller *controller) {
    dshot_loop_async_start(controller);
    dshot_loop_async_complete(controller);
}

void dshot_mark_activity(struct dshot_controller *controller) {
    controller->command_last_time = get_absolute_time();
}

/*
 * Compute a 16-bit DShot frame from throttle/command value.
 * Format: [11-bit value][1-bit telemetry][4-bit CRC]
 * CRC is inverted for bidirectional DShot (signals ESC to respond on same wire).
 */
static uint16_t dshot_compute_frame(uint16_t throttle, int telemetry) {
    uint16_t value = (throttle << 1) | telemetry;

    uint16_t crc = value ^ (value >> 4) ^ (value >> 8);
    crc = ~crc;

    return (value << 4) | (crc & 0x0F);
}

void dshot_command(struct dshot_controller *controller, uint16_t channel, uint16_t command,
                   uint8_t repeat_count) {
    if (channel >= controller->num_channels) {
        return;
    }

    struct dshot_motor *motor = &controller->motor[channel];

    motor->frame = dshot_compute_frame(command, 1);
    motor->current_command = command;
    motor->command_counter = repeat_count;

    if (command == DSHOT_EXTENDED_TELEMETRY_DISABLE) {
        motor->edt_enabled = false;
        motor->telemetry_types = 0;
    }

    dshot_mark_activity(controller);
}

void dshot_throttle(struct dshot_controller *controller, uint16_t channel, uint16_t throttle) {
    if (channel >= controller->num_channels) {
        return;
    }

    struct dshot_motor *motor = &controller->motor[channel];

    if (motor->last_throttle_value == throttle) {
        if (motor->command_counter == 0) {
            motor->frame = motor->last_throttle_frame;
        }
        return;
    }

    motor->last_throttle_value = throttle;
    motor->last_throttle_frame = dshot_compute_frame(throttle, 0);
    if (motor->command_counter == 0) {
        motor->frame = motor->last_throttle_frame;
    }
}

bool dshot_is_telemetry_active(const struct dshot_controller *controller) {
    for (int i = 0; i < controller->num_channels; i++) {
        if (!(controller->motor[i].telemetry_types & (1 << DSHOT_TELEMETRY_TYPE_ERPM))) {
            return false;
        }
    }
    return controller->num_channels > 0;
}

int16_t dshot_get_telemetry_quality_percent(const struct dshot_controller *controller,
                                            uint8_t channel) {
    if (channel >= controller->num_channels) {
        return 0;
    }
    const struct dshot_motor *motor = &controller->motor[channel];
    if (!(motor->telemetry_types & (1 << DSHOT_TELEMETRY_TYPE_ERPM))) {
        return 0;
    }
    if (motor->quality.packet_count_sum == 0) {
        return 0;
    }
    uint32_t valid = motor->quality.packet_count_sum - motor->quality.invalid_count_sum;
    return (int16_t)(valid * 10000 / motor->quality.packet_count_sum);
}

uint32_t dshot_get_last_erpm_age_ms(const struct dshot_controller *controller, uint8_t channel,
                                    uint32_t now_ms) {
    if (channel >= controller->num_channels) {
        return UINT32_MAX;
    }
    const struct dshot_motor *motor = &controller->motor[channel];
    if (!(motor->telemetry_types & (1 << DSHOT_TELEMETRY_TYPE_ERPM))) {
        return UINT32_MAX;
    }
    return now_ms - motor->last_erpm_ms;
}
