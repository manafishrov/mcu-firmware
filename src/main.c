#include "am32/bootloader.h"
#include "dshot/control.h"
#include "dshot/dshot.h"
#include "dshot/telemetry_usb.h"
#include "esc_firmware/update.h"
#include "log.h"
#include "motors.h"
#include "pwm/control.h"
#include "pwm/pwm.h"
#include "runtime_config.h"
#include "usb_comm.h"
#include <hardware/pio.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DSHOT_PIO pio0
#define DSHOT_SM_0 0
#define DSHOT_SM_1 1

#define INPUT_PACKET_SIZE USB_INPUT_PACKET_SIZE(NUM_MOTORS)
#define QUALITY_WARN_THRESHOLD 5000
#define QUALITY_REPORT_INTERVAL_MS 100
#define DSHOT_TELEMETRY_WARNING_DELAY_MS 500
#define ESC_PROTOCOL_DETECTION_RESET_MS 2100
#define PWM_DETECTION_SETTLE_MS 1000
#define ESC_VERSION_DISCOVERY_ATTEMPTS 3
#define ESC_VERSION_DISCOVERY_RETRY_MS 250

static uint16_t command_values[NUM_MOTORS] = {CMD_THROTTLE_NEUTRAL};
static absolute_time_t last_comm_time;
static bool comm_timed_out = true;

static absolute_time_t next_quality_report_time;
static bool edt_enable_scheduled[NUM_MOTORS] = {false};
static absolute_time_t edt_enable_time[NUM_MOTORS];
static bool quality_warned[NUM_MOTORS] = {false};

static struct pwm_controller pwm_controller;
static struct dshot_controller dshot_controller0;
static struct dshot_controller dshot_controller1;
static dshot_telemetry_context_t dshot_context0 = {.controller_base_global_id = 0};
static dshot_telemetry_context_t dshot_context1 = {.controller_base_global_id = NUM_MOTORS_0};
static bool pwm_initialized = false;
static bool dshot_initialized = false;
static bool runtime_config_received = false;
static bool protocol_initialized = false;
static bool all_motor_telemetry_seen = false;
static bool dshot_telemetry_warning_pending = false;
static absolute_time_t dshot_telemetry_warning_time;
// A failed update can leave one or more ESCs partially programmed. Keep every
// ESC in the bootloader until a complete retry succeeds; starting DShot here
// could execute an incomplete application.
static bool esc_firmware_recovery_mode = false;
static bool esc_version_discovery_active = false;
static uint8_t esc_version_discovery_attempts = 0;
static absolute_time_t next_esc_version_discovery_time;

static mcu_runtime_config_t current_config = {0};

static void send_quality_reports(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct dshot_controller *ctrl;
        int channel;
        dshot_get_motor_controller(i, &ctrl, &channel, &dshot_controller0, &dshot_controller1);
        int16_t quality = dshot_get_telemetry_quality_percent(ctrl, channel);
        uint32_t erpm_age_ms = dshot_get_last_erpm_age_ms(ctrl, channel, now_ms);
        int32_t reported_quality =
            erpm_age_ms == UINT32_MAX ? TELEMETRY_SIGNAL_QUALITY_UNAVAILABLE : (int32_t)quality;
        dshot_telemetry_usb_send(i, TELEMETRY_TYPE_SIGNAL_QUALITY, reported_quality);

        if (quality < QUALITY_WARN_THRESHOLD && !quality_warned[i]) {
            quality_warned[i] = true;
            if (erpm_age_ms == UINT32_MAX) {
                log_warnf("Motor %d: DShot telemetry %d.%02d%%, types=0x%02x, last eRPM=never, "
                          "cause=%s",
                          i, quality / 100, quality % 100, ctrl->motor[channel].telemetry_types,
                          dshot_dominant_failure_name(&ctrl->motor[channel].stats));
            } else {
                log_warnf("Motor %d: DShot telemetry %d.%02d%%, types=0x%02x, last eRPM=%ums "
                          "ago, cause=%s",
                          i, quality / 100, quality % 100, ctrl->motor[channel].telemetry_types,
                          erpm_age_ms, dshot_dominant_failure_name(&ctrl->motor[channel].stats));
            }
        } else if (quality > QUALITY_WARN_THRESHOLD + 1000 && quality_warned[i]) {
            quality_warned[i] = false;
        }
    }
}

static void set_all_commands_neutral(void) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        command_values[i] = CMD_THROTTLE_NEUTRAL;
    }
}

static bool all_commands_neutral(void) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (command_values[i] != CMD_THROTTLE_NEUTRAL) {
            return false;
        }
    }
    return true;
}

static void deinit_protocol(thruster_protocol_t protocol) {
    if (protocol == THRUSTER_PROTOCOL_DSHOT && dshot_initialized) {
        dshot_telemetry_usb_flush();
        dshot_controller_deinit(&dshot_controller0);
        dshot_controller_deinit(&dshot_controller1);
        dshot_telemetry_usb_reset();
        dshot_initialized = false;
        dshot_telemetry_warning_pending = false;
    } else if (protocol == THRUSTER_PROTOCOL_PWM && pwm_initialized) {
        pwm_controller_deinit(&pwm_controller);
        pwm_initialized = false;
    }
}

static void init_pwm_protocol(void) {
    uint pins[NUM_MOTORS] = {MOTOR0_PIN_BASE,     MOTOR0_PIN_BASE + 1, MOTOR0_PIN_BASE + 2,
                             MOTOR0_PIN_BASE + 3, MOTOR1_PIN_BASE,     MOTOR1_PIN_BASE + 1,
                             MOTOR1_PIN_BASE + 2, MOTOR1_PIN_BASE + 3};
    pwm_controller_init(&pwm_controller, pins, NUM_MOTORS,
                        pwm_translate_throttle(CMD_THROTTLE_NEUTRAL));
    pwm_initialized = true;

    // PWM-capable ESCs need multiple 50 Hz neutral frames to detect the
    // waveform and arm. Do not acknowledge the configuration until every
    // channel has had enough time to observe those frames.
    sleep_ms(PWM_DETECTION_SETTLE_MS);
}

static void init_dshot_protocol(uint16_t dshot_speed, bool persist_3d_mode) {
    dshot_controller_reset_calibration();
    dshot_telemetry_usb_init();
    dshot_controller_init(&dshot_controller0, dshot_speed, DSHOT_PIO, DSHOT_SM_0, MOTOR0_PIN_BASE,
                          NUM_MOTORS_0);
    dshot_register_telemetry_cb(&dshot_controller0, dshot_telemetry_callback, &dshot_context0);

    dshot_controller_init(&dshot_controller1, dshot_speed, DSHOT_PIO, DSHOT_SM_1, MOTOR1_PIN_BASE,
                          NUM_MOTORS_1);
    dshot_register_telemetry_cb(&dshot_controller1, dshot_telemetry_callback, &dshot_context1);

    for (int i = 0; i < NUM_MOTORS; ++i) {
        edt_enable_scheduled[i] = false;
        edt_enable_time[i] = get_absolute_time();
        quality_warned[i] = false;
    }
    dshot_send_commands(command_values, &dshot_controller0, &dshot_controller1);
    dshot_run_frame_cycles(&dshot_controller0, &dshot_controller1, NUM_MOTORS * 4);
    dshot_send_command_to_all(&dshot_controller0, &dshot_controller1, DSHOT_CMD_3D_MODE_ON, 10);
    if (persist_3d_mode) {
        dshot_send_command_to_all(&dshot_controller0, &dshot_controller1, DSHOT_CMD_SAVE_SETTINGS,
                                  10);
    }
    dshot_send_command_to_all(&dshot_controller0, &dshot_controller1,
                              DSHOT_EXTENDED_TELEMETRY_ENABLE, 10);
    dshot_initialized = true;
    dshot_telemetry_warning_time =
        delayed_by_ms(get_absolute_time(), DSHOT_TELEMETRY_WARNING_DELAY_MS);
    next_quality_report_time = dshot_telemetry_warning_time;
    dshot_telemetry_warning_pending = true;
}

static void init_current_protocol(bool persist_3d_mode) {
    if (current_config.protocol == THRUSTER_PROTOCOL_DSHOT) {
        init_dshot_protocol(current_config.dshot_speed, persist_3d_mode);
    } else {
        init_pwm_protocol();
    }
    protocol_initialized = true;
}

static bool dshot_telemetry_ready(void) {
    return dshot_is_telemetry_active(&dshot_controller0) &&
           dshot_is_telemetry_active(&dshot_controller1);
}

static void request_esc_firmware_versions_if_idle(void) {
    if (current_config.protocol != THRUSTER_PROTOCOL_DSHOT || !dshot_initialized ||
        !protocol_initialized || !all_commands_neutral() || esc_firmware_update_receiving()) {
        return;
    }

    dshot_telemetry_usb_begin_esc_version_discovery();
    esc_version_discovery_active = true;
    esc_version_discovery_attempts = 0;
    next_esc_version_discovery_time = get_absolute_time();
}

static void service_esc_version_discovery(void) {
    if (!esc_version_discovery_active) {
        return;
    }
    if (dshot_telemetry_usb_all_esc_versions_reported()) {
        esc_version_discovery_active = false;
        return;
    }
    if (esc_version_discovery_attempts >= ESC_VERSION_DISCOVERY_ATTEMPTS) {
        esc_version_discovery_active = false;
        uint8_t reported = dshot_telemetry_usb_esc_versions_reported_count();
        if (reported > 0) {
            log_warnf("ESC firmware version discovery received %u of %u responses", reported,
                      NUM_MOTORS);
        }
        return;
    }
    if (!all_commands_neutral()) {
        return;
    }

    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(next_esc_version_discovery_time, now) < 0) {
        return;
    }
    dshot_send_command_to_all(&dshot_controller0, &dshot_controller1,
                              DSHOT_EXTENDED_TELEMETRY_ENABLE, 10);
    esc_version_discovery_attempts++;
    next_esc_version_discovery_time = delayed_by_ms(now, ESC_VERSION_DISCOVERY_RETRY_MS);
}

static void log_missing_dshot_telemetry(void) {
    uint8_t missing = dshot_missing_telemetry_mask(&dshot_controller0, &dshot_controller1);
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if ((missing & (1u << i)) != 0) {
            struct dshot_controller *ctrl;
            int channel;
            dshot_get_motor_controller(i, &ctrl, &channel, &dshot_controller0, &dshot_controller1);
            log_warnf("Motor %d has not reported eRPM telemetry (%s)", i,
                      dshot_dominant_failure_name(&ctrl->motor[channel].stats));
        }
    }
}

static void hold_neutral_before_switch(void) {
    set_all_commands_neutral();
    if (current_config.protocol == THRUSTER_PROTOCOL_DSHOT && dshot_initialized) {
        dshot_send_commands(command_values, &dshot_controller0, &dshot_controller1);
        for (int i = 0; i < 120; ++i) {
            dshot_loop(&dshot_controller0);
            dshot_loop(&dshot_controller1);
        }
        dshot_telemetry_usb_flush();
    } else if (current_config.protocol == THRUSTER_PROTOCOL_PWM && pwm_initialized) {
        for (int i = 0; i < NUM_MOTORS; ++i) {
            pwm_set_throttle(&pwm_controller, i, pwm_translate_throttle(CMD_THROTTLE_NEUTRAL));
        }
        sleep_ms(1000);
    }
}

static void apply_runtime_config(mcu_runtime_config_t config) {
    mcu_runtime_config_validate(&config);

    bool switching = runtime_config_received && (dshot_initialized || pwm_initialized);
    bool reset_detector = runtime_config_received &&
                          mcu_runtime_config_requires_detector_reset(&current_config, &config);
    bool entering_dshot =
        config.protocol == THRUSTER_PROTOCOL_DSHOT &&
        (!runtime_config_received || current_config.protocol != THRUSTER_PROTOCOL_DSHOT);
    if (switching) {
        hold_neutral_before_switch();
    }

    protocol_initialized = false;
    all_motor_telemetry_seen = false;

    if (runtime_config_received) {
        deinit_protocol(current_config.protocol);
    }

    if (reset_detector) {
        // The ESC keeps the detected input decoder until it has seen two seconds
        // without a valid signal while unarmed. This applies both to protocol
        // changes and to DShot baud-rate changes after its frame timing has
        // been calibrated. Keep every output low for slightly longer so all
        // ESCs return to detection before the new waveform starts.
        sleep_ms(ESC_PROTOCOL_DETECTION_RESET_MS);
    }

    esc_firmware_update_reset();
    current_config = config;

    init_current_protocol(entering_dshot);

    runtime_config_received = true;
    comm_timed_out = true;
    mcu_runtime_config_send_status(&current_config);
    request_esc_firmware_versions_if_idle();
}

static void handle_command_packet(uint8_t *command_buf) {
    if (!runtime_config_received) {
        return;
    }

    if (!usb_parse_packet(command_buf, INPUT_PACKET_SIZE, command_values, NUM_MOTORS,
                          &last_comm_time)) {
        return;
    }

    if (!protocol_initialized) {
        set_all_commands_neutral();
        return;
    }

    if (current_config.protocol == THRUSTER_PROTOCOL_DSHOT) {
        dshot_mark_activity(&dshot_controller0);
        dshot_mark_activity(&dshot_controller1);
    }

    if (comm_timed_out) {
        comm_timed_out = false;
        if (current_config.protocol == THRUSTER_PROTOCOL_DSHOT) {
            log_infof("DShot%u, 8 motors, USB comm active", current_config.dshot_speed);
        } else {
            log_info("PWM, 8 motors, USB comm active");
        }
    }
}

static void handle_config_packet(uint8_t *config_buf) {
    mcu_runtime_config_t new_config;
    if (!mcu_runtime_config_parse_packet(config_buf, USB_CONFIG_PACKET_SIZE, &new_config)) {
        return;
    }

    if (esc_firmware_recovery_mode) {
        log_warn("Ignoring runtime config while ESC firmware recovery is required");
        esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_RECOVERY_REQUIRED, UINT8_MAX,
                                        ESC_FIRMWARE_UPDATE_ERROR_NONE, 1);
        return;
    }

    if (runtime_config_received && new_config.protocol == current_config.protocol &&
        new_config.dshot_speed == current_config.dshot_speed) {
        if (protocol_initialized) {
            mcu_runtime_config_send_status(&current_config);
            request_esc_firmware_versions_if_idle();
        }
        return;
    }

    if (!all_commands_neutral()) {
        log_warn("Ignoring protocol change while thrusters active");
        mcu_runtime_config_send_status(&current_config);
        return;
    }

    apply_runtime_config(new_config);
    log_infof("Active thruster protocol: %s @ %u",
              mcu_runtime_config_protocol_name(current_config.protocol),
              current_config.dshot_speed);
}

static bool runtime_dshot_ready(void) {
    return runtime_config_received && current_config.protocol == THRUSTER_PROTOCOL_DSHOT &&
           dshot_initialized && protocol_initialized;
}

static void fail_esc_firmware_update(esc_firmware_update_error_t error, uint32_t value) {
    esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_FAILED, UINT8_MAX, error, value);
    esc_firmware_update_reset();
}

static void handle_esc_firmware_begin(bool recovery_request) {
    if (esc_firmware_recovery_mode && !recovery_request) {
        esc_firmware_update_reset();
        esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_RECOVERY_REQUIRED, UINT8_MAX,
                                        ESC_FIRMWARE_UPDATE_ERROR_NONE, 1);
        return;
    }

    if (!runtime_dshot_ready()) {
        if (recovery_request) {
            esc_version_discovery_active = false;
            esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_READY, UINT8_MAX,
                                            ESC_FIRMWARE_UPDATE_ERROR_NONE,
                                            esc_firmware_update_image_size());
            return;
        }
        fail_esc_firmware_update(ESC_FIRMWARE_UPDATE_ERROR_NOT_DSHOT, 0);
        return;
    }

    if (!all_commands_neutral()) {
        fail_esc_firmware_update(ESC_FIRMWARE_UPDATE_ERROR_THRUSTERS_ACTIVE, 0);
        return;
    }

    // Keep updater acknowledgements isolated on USB while the image is
    // staged. DShot continues sending neutral frames, but ordinary
    // telemetry and version discovery remain quiet until the transaction
    // completes or is aborted.
    esc_version_discovery_active = false;
    dshot_telemetry_usb_reset();
    esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_READY, UINT8_MAX,
                                    ESC_FIRMWARE_UPDATE_ERROR_NONE,
                                    esc_firmware_update_image_size());
}

static bool esc_firmware_commit_is_safe(bool recovery_transaction,
                                        esc_firmware_update_error_t *error) {
    if (!recovery_transaction && !runtime_dshot_ready()) {
        fail_esc_firmware_update(ESC_FIRMWARE_UPDATE_ERROR_NOT_DSHOT, 0);
        return false;
    }
    if (!esc_firmware_update_validate_image(error)) {
        fail_esc_firmware_update(*error, esc_firmware_update_received_size());
        return false;
    }
    if (runtime_config_received && !all_commands_neutral()) {
        fail_esc_firmware_update(ESC_FIRMWARE_UPDATE_ERROR_THRUSTERS_ACTIVE, 0);
        return false;
    }
    return true;
}

static void prepare_for_esc_firmware_flash(bool was_recovery) {
    // Discard telemetry accumulated while USB output was suspended so it
    // cannot delay bootloader progress statuses after COMMIT.
    if (dshot_initialized) {
        dshot_telemetry_usb_reset();
    }
    if (!was_recovery && protocol_initialized) {
        hold_neutral_before_switch();
    }
    protocol_initialized = false;
    all_motor_telemetry_seen = false;
    if (runtime_config_received) {
        deinit_protocol(current_config.protocol);
    }
    esc_version_discovery_active = false;
    esc_firmware_recovery_mode = true;
}

static void finish_successful_esc_firmware_flash(void) {
    esc_firmware_recovery_mode = false;
    if (runtime_config_received) {
        init_current_protocol(false);
    }
    esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_COMPLETE, UINT8_MAX,
                                    ESC_FIRMWARE_UPDATE_ERROR_NONE,
                                    esc_firmware_update_image_size());
    if (dshot_initialized) {
        request_esc_firmware_versions_if_idle();
    }
}

static void finish_failed_esc_firmware_flash(bool was_recovery, bool modified, uint8_t failed_motor,
                                             esc_firmware_update_error_t error) {
    bool recovery_required = was_recovery || modified;
    esc_firmware_recovery_mode = recovery_required;
    if (!recovery_required && runtime_config_received) {
        init_current_protocol(false);
    }
    esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_FAILED, failed_motor, error,
                                    recovery_required ? 1u : 0u);
    if (recovery_required) {
        log_warn("ESC firmware update failed; keeping ESCs in bootloader for a safe retry");
    } else {
        log_warn("ESC firmware update stopped before any ESC was modified");
    }
}

static void commit_esc_firmware_update(esc_firmware_update_error_t *error) {
    bool recovery_transaction = esc_firmware_update_recovery_requested();
    if (!esc_firmware_commit_is_safe(recovery_transaction, error)) {
        return;
    }

    bool was_recovery = esc_firmware_recovery_mode || recovery_transaction;
    prepare_for_esc_firmware_flash(was_recovery);

    uint8_t failed_motor = UINT8_MAX;
    bool modified = false;
    bool flashed =
        am32_bootloader_flash_all(esc_firmware_update_image(), esc_firmware_update_image_size(),
                                  error, &failed_motor, &modified);
    if (flashed) {
        finish_successful_esc_firmware_flash();
    } else {
        finish_failed_esc_firmware_flash(was_recovery, modified, failed_motor, *error);
    }
    esc_firmware_update_reset();
}

static void handle_esc_firmware_control_packet(const uint8_t *packet) {
    esc_firmware_update_command_t command;
    esc_firmware_update_error_t error;
    if (!esc_firmware_update_parse_control(packet, &command, &error)) {
        esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_FAILED, UINT8_MAX, error, 0);
        return;
    }

    if (command == ESC_FIRMWARE_UPDATE_COMMAND_ABORT) {
        esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_ABORTED, UINT8_MAX,
                                        ESC_FIRMWARE_UPDATE_ERROR_NONE, 0);
        request_esc_firmware_versions_if_idle();
        return;
    }

    bool recovery_request = command == ESC_FIRMWARE_UPDATE_COMMAND_RECOVER_BEGIN;
    if (command == ESC_FIRMWARE_UPDATE_COMMAND_BEGIN || recovery_request) {
        handle_esc_firmware_begin(recovery_request);
        return;
    }
    commit_esc_firmware_update(&error);
}

static void handle_esc_firmware_data_packet(const uint8_t *packet) {
    esc_firmware_update_error_t error;
    if (!esc_firmware_update_receive_data(packet, &error)) {
        esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_FAILED, UINT8_MAX, error,
                                        esc_firmware_update_received_size());
        esc_firmware_update_reset();
        request_esc_firmware_versions_if_idle();
        return;
    }
    esc_firmware_update_send_status(ESC_FIRMWARE_UPDATE_STATUS_RECEIVED, UINT8_MAX,
                                    ESC_FIRMWARE_UPDATE_ERROR_NONE,
                                    esc_firmware_update_received_size());
}

static void service_dshot_protocol(void) {
    bool esc_firmware_upload_active = esc_firmware_update_receiving();
    if (esc_firmware_upload_active) {
        set_all_commands_neutral();
    }
    dshot_send_commands(command_values, &dshot_controller0, &dshot_controller1);
    dshot_enable_edt_if_idle(command_values, edt_enable_scheduled, edt_enable_time,
                             &dshot_controller0, &dshot_controller1);
    dshot_loop(&dshot_controller0);
    dshot_loop(&dshot_controller1);
    if (esc_firmware_upload_active) {
        return;
    }

    service_esc_version_discovery();
    if (dshot_quality_report_due(&next_quality_report_time, QUALITY_REPORT_INTERVAL_MS,
                                 get_absolute_time())) {
        send_quality_reports();
    }
    dshot_telemetry_usb_flush();
    if (!all_motor_telemetry_seen && dshot_telemetry_ready()) {
        all_motor_telemetry_seen = true;
        dshot_telemetry_warning_pending = false;
        request_esc_firmware_versions_if_idle();
        log_info("DShot telemetry active on all motors");
    } else if (!all_motor_telemetry_seen && dshot_telemetry_warning_pending &&
               absolute_time_diff_us(get_absolute_time(), dshot_telemetry_warning_time) <= 0) {
        dshot_telemetry_warning_pending = false;
        log_warn("DShot initialized, but telemetry is not active on every motor");
        log_missing_dshot_telemetry();
    }
}

static void service_pwm_protocol(void) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        pwm_set_throttle(&pwm_controller, i, pwm_translate_throttle(command_values[i]));
    }
}

int main(void) {
    stdio_init_all();
    log_init();

    set_all_commands_neutral();
    last_comm_time = get_absolute_time();
    log_info("Waiting for runtime config from main firmware");

    static uint8_t command_buf[INPUT_PACKET_SIZE];
    static uint8_t config_buf[USB_CONFIG_PACKET_SIZE];
    static uint8_t esc_firmware_control_buf[ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE];
    static uint8_t esc_firmware_data_buf[ESC_FIRMWARE_USB_DATA_PACKET_SIZE];
    usb_packet_reader_t readers[] = {
        {USB_INPUT_START_BYTE, command_buf, INPUT_PACKET_SIZE, 0, USB_PACKET_COMMAND, 0},
        {USB_CONFIG_START_BYTE, config_buf, USB_CONFIG_PACKET_SIZE, 0, USB_PACKET_CONFIG, 0},
        {ESC_FIRMWARE_USB_CONTROL_START_BYTE, esc_firmware_control_buf,
         ESC_FIRMWARE_USB_CONTROL_PACKET_SIZE, 0, USB_PACKET_ESC_FIRMWARE_CONTROL, 0},
        {ESC_FIRMWARE_USB_DATA_START_BYTE, esc_firmware_data_buf, ESC_FIRMWARE_USB_DATA_PACKET_SIZE,
         0, USB_PACKET_ESC_FIRMWARE_DATA, 0},
    };
    esc_firmware_update_reset();

    while (true) {
        usb_packet_kind_t packet_kind = usb_poll(readers, sizeof(readers) / sizeof(readers[0]));

        if (packet_kind == USB_PACKET_COMMAND) {
            handle_command_packet(command_buf);
        } else if (packet_kind == USB_PACKET_CONFIG) {
            handle_config_packet(config_buf);
        } else if (packet_kind == USB_PACKET_ESC_FIRMWARE_CONTROL) {
            handle_esc_firmware_control_packet(esc_firmware_control_buf);
        } else if (packet_kind == USB_PACKET_ESC_FIRMWARE_DATA) {
            handle_esc_firmware_data_packet(esc_firmware_data_buf);
        }

        usb_check_timeout(last_comm_time, command_values, NUM_MOTORS, CMD_THROTTLE_NEUTRAL,
                          &comm_timed_out);

        if (!runtime_config_received) {
            continue;
        }

        if (esc_firmware_recovery_mode) {
            continue;
        }

        if (current_config.protocol == THRUSTER_PROTOCOL_DSHOT) {
            service_dshot_protocol();
        } else {
            service_pwm_protocol();
        }
    }
}
