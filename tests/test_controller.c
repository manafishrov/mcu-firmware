#include "control/controller.h"
#include "unity/unity.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define TEST_RAD_PER_DEG 0.01745329251994329577f

static control_state_t state;
static control_output_t output;
static const control_sample_t stationary = {.accel = {0.0f, 0.0f, -9.81f}, .gyro = {0}};
static const control_sample_t gyro_only = {.accel = {0}, .gyro = {0}};

static void identity_settings(void) {
    control_init(&state);
    for (unsigned i = 0; i < 8; ++i) {
        state.settings.allocation[i][i] = 1.0f;
    }
    for (unsigned i = 0; i < 3; ++i) {
        state.settings.power[i] = 100.0f;
    }
    memset(&output, 0, sizeof(output));
}

static void command_zero(bool stabilization, bool depth_hold) {
    const float direction[8] = {0};
    control_command(&state, direction, 1.0f / 60.0f, stabilization, depth_hold);
}

static void test_init_and_settings_validation(void) {
    identity_settings();
    TEST_ASSERT_TRUE(control_validate_settings(&state.settings));
    TEST_ASSERT_FALSE(control_validate_settings(NULL));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, state.current_q[3]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, state.desired_q[3]);
    control_settings_t candidate = state.settings;
    candidate.identifiers[1] = 0; /* np.take permits duplicates. */
    candidate.roll.kp = -1.0f;    /* Finite gains are not silently constrained. */
    TEST_ASSERT_TRUE(control_validate_settings(&candidate));
    candidate.identifiers[1] = 8;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    candidate = state.settings;
    candidate.spin[0] = 0;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    candidate = state.settings;
    candidate.power[2] = 100.01f;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    candidate = state.settings;
    candidate.power[0] = -0.01f;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    candidate = state.settings;
    candidate.nullspace_count = 9;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    candidate = state.settings;
    candidate.nullspace[7][7] = NAN;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    candidate.nullspace[7][7] = 0.1f;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    candidate.nullspace_count = 8;
    TEST_ASSERT_TRUE(control_validate_settings(&candidate));
    candidate.allocation[3][5] = INFINITY;
    TEST_ASSERT_FALSE(control_validate_settings(&candidate));
    control_apply_settings(&state, &candidate);
    TEST_ASSERT_EQUAL_UINT32(0, state.settings.nullspace_count);
}

static void test_all_axis_and_coefficient_floats_validated(void) {
    identity_settings();
    control_axis_t *axes[] = {&state.settings.roll, &state.settings.pitch, &state.settings.yaw,
                              &state.settings.depth};
    for (unsigned i = 0; i < 4; ++i) {
        axes[i]->kp = NAN;
        TEST_ASSERT_FALSE(control_validate_settings(&state.settings));
        axes[i]->kp = 0.0f;
        axes[i]->ki = NAN;
        TEST_ASSERT_FALSE(control_validate_settings(&state.settings));
        axes[i]->ki = 0.0f;
        axes[i]->kd = NAN;
        TEST_ASSERT_FALSE(control_validate_settings(&state.settings));
        axes[i]->kd = 0.0f;
        axes[i]->rate = NAN;
        TEST_ASSERT_FALSE(control_validate_settings(&state.settings));
        axes[i]->rate = 0.0f;
    }
    for (unsigned i = 0; i < 3; ++i) {
        state.settings.coefficients[i] = INFINITY;
        TEST_ASSERT_FALSE(control_validate_settings(&state.settings));
        state.settings.coefficients[i] = 0.0f;
    }
    TEST_ASSERT_TRUE(control_validate_settings(&state.settings));
}

static void test_gyro_exponential_uses_500hz_dt_and_normalizes(void) {
    identity_settings();
    control_sample_t sample = {.accel = {0}, .gyro = {0, 0, 1}};
    for (unsigned i = 0; i < 500; ++i) {
        control_step(&state, &sample, 0.002f, &output);
    }
    TEST_ASSERT_FLOAT_WITHIN(5.0e-6f, sinf(0.5f), output.current_q[2]);
    TEST_ASSERT_FLOAT_WITHIN(5.0e-6f, cosf(0.5f), output.current_q[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.current_q[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.current_q[1]);
}

static void test_step_dt_clamp_is_separate_from_command_clamp(void) {
    const float proposed[] = {-1.0f, 1.0f, NAN, INFINITY};
    const float expected[] = {0.001f, 0.02f, 0.002f, 0.002f};
    const control_sample_t sample = {.accel = {0}, .gyro = {1, 0, 0}};
    for (unsigned i = 0; i < 4; ++i) {
        identity_settings();
        control_step(&state, &sample, proposed[i], &output);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, sinf(expected[i] / 2.0f), output.current_q[0]);
    }
    const float direction[8] = {0, 0, 1, 0, 0, 0, 0, 0};
    const float source[] = {-1.0f, 1.0f, NAN};
    const float source_expected[] = {1.0f / 120.0f, 1.0f / 6.0f, 1.0f / 60.0f};
    for (unsigned i = 0; i < 3; ++i) {
        identity_settings();
        state.settings.depth.rate = 1.0f;
        control_command(&state, direction, source[i], false, true);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, source_expected[i], state.desired_depth);
    }
}

static void test_mahony_ned_cross_order_integral_and_gyro_only_fallback(void) {
    identity_settings();
    const control_sample_t sample = {.accel = {0, 1, -1}, .gyro = {0}};
    control_step(&state, &sample, 0.002f, &output);
    float error = -1.0f / sqrtf(2.0f);
    float integral = error * 0.05f * 0.002f;
    float omega = 1.5f * error + integral;
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, integral, state.ahrs_integral[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, sinf(omega * 0.001f), state.current_q[0]);
    float before[4];
    memcpy(before, state.current_q, sizeof(before));
    control_step(&state, &gyro_only, 0.002f, &output);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(before, state.current_q, 4);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, integral, state.ahrs_integral[0]);
    control_sample_t invalid_accel = {.accel = {NAN, 0, 0}, .gyro = {0, 0, 1}};
    control_step(&state, &invalid_accel, 0.002f, &output);
    TEST_ASSERT_TRUE(isfinite(state.current_q[2]));
    TEST_ASSERT_TRUE(state.current_q[2] > 0.0f);
}

static void test_gyro_rejection_preserves_original_pid_derivative(void) {
    identity_settings();
    state.settings.roll.kd = 0.1f;
    state.settings.pitch.kd = 0.2f;
    command_zero(true, false);
    const control_sample_t excessive = {.accel = {0}, .gyro = {20, 2, 0}};
    control_step(&state, &excessive, 0.002f, &output);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, state.current_q[3]);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, state.gyro[0]);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, state.gyro[1]);
    TEST_ASSERT_EQUAL_UINT16(800, output.motors[5]);
    TEST_ASSERT_EQUAL_UINT16(960, output.motors[3]);
}

static void test_absolute_target_normalization_rejection_and_enable_edge(void) {
    identity_settings();
    const float q[4] = {0, 0, 2, 2};
    TEST_ASSERT_TRUE(control_set_attitude(&state, q));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, 1.0f / sqrtf(2.0f), state.desired_q[2]);
    state.attitude_integral[0] = 0.5f;
    const float invalid[][4] = {
        {0, 0, 0, 0}, {NAN, 0, 0, 1}, {0, INFINITY, 0, 1}, {1.0e-20f, 0, 0, 0}};
    for (unsigned i = 0; i < 4; ++i) {
        TEST_ASSERT_FALSE(control_set_attitude(&state, invalid[i]));
    }
    TEST_ASSERT_EQUAL_FLOAT(0.5f, state.attitude_integral[0]);
    command_zero(true, false);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, state.desired_q[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.attitude_integral[0]);
    TEST_ASSERT_TRUE(control_set_attitude(&state, q));
    state.attitude_integral[0] = 0.5f;
    command_zero(true, false);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, 1.0f / sqrtf(2.0f), state.desired_q[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, state.attitude_integral[0]);
}

static void test_pid_shortest_rotation_threshold_clips_and_no_antiwindup(void) {
    identity_settings();
    state.settings.roll.kp = 10.0f;
    state.settings.roll.ki = 10.0f;
    state.settings.power[2] = 1.0f;
    command_zero(true, false);
    const float q[4] = {sinf(1.5f), 0, 0, cosf(1.5f)};
    TEST_ASSERT_TRUE(control_set_attitude(&state, q));
    float direction[8] = {0, 0, 0, 0.2f, 0, 0, 0, 0};
    control_command(&state, direction, 1.0f / 60.0f, true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.attitude_integral[0]);
    direction[3] = 0.1999f;
    control_command(&state, direction, 1.0f / 60.0f, true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, 0.006f, state.attitude_integral[0]);
    TEST_ASSERT_EQUAL_UINT16(1010, output.motors[5]);
    for (unsigned i = 0; i < 1000; ++i) {
        control_step(&state, &stationary, 0.002f, &output);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 100.0f * TEST_RAD_PER_DEG, state.attitude_integral[0]);
    /* The antipodal quaternion represents the same shortest error. */
    const float negative_q[4] = {-q[0], 0, 0, -q[3]};
    TEST_ASSERT_TRUE(control_set_attitude(&state, negative_q));
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_EQUAL_UINT16(1010, output.motors[5]);
}

static void test_targets_and_depth_pid_advance_only_on_commands(void) {
    identity_settings();
    state.settings.yaw.rate = 60.0f;
    state.settings.depth = (control_axis_t){.kp = 2.0f, .ki = 3.0f, .kd = 4.0f, .rate = 1.0f};
    control_pressure(&state, 4.0f, 0.5f);
    control_set_depth(&state, 6.0f);
    const float direction[8] = {0, 0, 0.5f, 0, 0.25f, 0, 0, 0};
    control_command(&state, direction, 1.0f / 60.0f, true, true);
    float error = 2.0f + 0.5f / 60.0f;
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 6.0f + 0.5f / 60.0f, state.desired_depth);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, error / 120.0f, state.depth_integral);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 2.0f * error + 3.0f * error / 120.0f - 2.0f,
                             state.depth_actuation);
    float target[4];
    memcpy(target, state.desired_q, sizeof(target));
    float depth = state.desired_depth;
    float integral = state.depth_integral;
    float actuation = state.depth_actuation;
    for (unsigned i = 0; i < 100; ++i) {
        control_step(&state, &stationary, 0.002f, &output);
    }
    control_pressure(&state, 5.0f, -1.0f);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(target, state.desired_q, 4);
    TEST_ASSERT_EQUAL_FLOAT(depth, state.desired_depth);
    TEST_ASSERT_EQUAL_FLOAT(integral, state.depth_integral);
    TEST_ASSERT_EQUAL_FLOAT(actuation, state.depth_actuation);
    control_command(&state, direction, 1.0f / 60.0f, true, true);
    TEST_ASSERT_TRUE(state.desired_q[2] > target[2]);
    TEST_ASSERT_TRUE(state.desired_depth > depth);
}

static void test_depth_pending_targets_disable_edges_and_negative_integration(void) {
    identity_settings();
    control_pressure(&state, 5.0f, 0.0f);
    control_set_depth(&state, -10.0f);
    command_zero(false, false);
    TEST_ASSERT_TRUE(state.has_pending_depth);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.desired_depth);
    command_zero(false, true);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.desired_depth);
    state.settings.depth.rate = 1.0f;
    float direction[8] = {0, 0, -1, 0, 0, 0, 0, 0};
    control_command(&state, direction, 1.0f / 60.0f, false, true);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-7f, -1.0f / 60.0f, state.desired_depth);
    control_set_depth(&state, 7.0f);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, state.desired_depth);
    command_zero(false, false);
    TEST_ASSERT_FALSE(state.has_pending_depth);
    command_zero(false, true);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, state.desired_depth);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.depth_integral);
}

static void test_depth_integral_relaxation_and_clip(void) {
    identity_settings();
    state.settings.depth.ki = 1.0f;
    control_set_depth(&state, 100.0f);
    command_zero(false, true);
    command_zero(false, true);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, state.depth_integral);
    const float direction[8] = {0, 0, 1, 0, 0, 0, 0, 0};
    control_pressure(&state, 200.0f, 0);
    control_command(&state, direction, 1.0f / 60.0f, false, true);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, state.depth_integral);
    for (unsigned i = 0; i < 4; ++i) {
        command_zero(false, true);
    }
    TEST_ASSERT_EQUAL_FLOAT(-3.0f, state.depth_integral);
}

static void test_allocation_power_work_reorder_spin_and_truncation(void) {
    identity_settings();
    state.settings.power[0] = 50.0f;
    state.settings.power[1] = 25.0f;
    state.settings.identifiers[0] = 6;
    state.settings.identifiers[1] = 6;
    state.settings.spin[1] = -1;
    const float direction[8] = {1, -1, 0.5f, -0.5f, 0.123f, -0.123f, 1, -1};
    control_command(&state, direction, 1.0f / 60.0f, false, false);
    control_step(&state, &stationary, 0.002f, &output);
    const uint16_t expected[8] = {1250, 750, 1250, 750, 1061, 938, 1250, 750};
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected, output.motors, 8);
    TEST_ASSERT_EQUAL_UINT8(65, output.work_percent);
    state.settings.allocation[0][0] = 10;
    state.settings.identifiers[0] = 0;
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_EQUAL_UINT16(2000, output.motors[0]);
}

static void test_user_and_regulator_limits_are_independent(void) {
    identity_settings();
    state.settings.power[0] = 0;
    state.settings.power[1] = 50;
    state.settings.power[2] = 10;
    state.settings.roll.kp = 10;
    command_zero(true, false);
    const float q[4] = {sinf(0.2f), 0, 0, cosf(0.2f)};
    TEST_ASSERT_TRUE(control_set_attitude(&state, q));
    const float direction[8] = {1, 0, 0, 0, 0, 0, 1, 0};
    control_command(&state, direction, 1.0f / 60.0f, true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_EQUAL_UINT16(1000, output.motors[0]);
    TEST_ASSERT_EQUAL_UINT16(1100, output.motors[5]);
    TEST_ASSERT_EQUAL_UINT16(1500, output.motors[6]);
    TEST_ASSERT_EQUAL_UINT8(30, output.work_percent);
}

static void test_movement_transform_coefficients_zero_ratios_and_yaw_removal(void) {
    identity_settings();
    /* 90 degree roll, plus arbitrary yaw: body forward is not world north. */
    state.current_q[0] = 0.5f;
    state.current_q[1] = 0.5f;
    state.current_q[2] = 0.5f;
    state.current_q[3] = 0.5f;
    state.settings.coefficients[0] = 2;
    state.settings.coefficients[1] = 0;
    state.settings.coefficients[2] = 4;
    state.settings.depth.kp = 1;
    control_set_depth(&state, 0.5f);
    const float direction[8] = {0.2f, 0.3f, 0, 0, 0, 0, 0, 0};
    control_command(&state, direction, 1.0f / 60.0f, false, true);
    control_step(&state, &gyro_only, 0.002f, &output);
    TEST_ASSERT_UINT16_WITHIN(1, 1200, output.motors[0]);
    TEST_ASSERT_UINT16_WITHIN(1, 1000, output.motors[1]);
    TEST_ASSERT_UINT16_WITHIN(1, 1000, output.motors[2]);
    state.settings.coefficients[1] = 2;
    control_step(&state, &gyro_only, 0.002f, &output);
    TEST_ASSERT_UINT16_WITHIN(1, 2000, output.motors[1]);
    TEST_ASSERT_UINT16_WITHIN(1, 850, output.motors[2]);
}

static void test_nullspace_initial_crossing_choice_and_sequential_order(void) {
    identity_settings();
    state.settings.nullspace_count = 2;
    state.settings.nullspace[0][0] = 1;
    state.settings.nullspace[0][1] = 1;
    state.settings.nullspace[1][0] = 1;
    state.settings.nullspace[1][1] = -1;
    command_zero(true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-9f, -0.003f, (float)state.nv_activation[0]);
    /* Second vector sees the first correction, not the original zero thrust. */
    TEST_ASSERT_FLOAT_WITHIN(1.0e-9f, -0.006f, (float)state.nv_activation[1]);
    TEST_ASSERT_EQUAL_UINT8(0, state.nv_deadzones[0]);
    TEST_ASSERT_EQUAL_UINT8(0, state.nv_deadzones[1]);
    TEST_ASSERT_EQUAL_UINT16(991, output.motors[0]);
    TEST_ASSERT_EQUAL_UINT16(1003, output.motors[1]);
    TEST_ASSERT_EQUAL_UINT8(0, output.work_percent);
}

static void test_nullspace_decay_only_once_per_command(void) {
    identity_settings();
    state.settings.nullspace_count = 1;
    state.settings.nullspace[0][0] = 1;
    state.nv_activation[0] = 0.05;
    state.nv_deadzones[0] = 1;
    const float direction[8] = {0.5f, 0, 0, 0, 0, 0, 0, 0};
    control_command(&state, direction, 1.0f / 60.0f, true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, 0.049f, (float)state.nv_activation[0]);
    for (unsigned i = 0; i < 10; ++i) {
        control_step(&state, &stationary, 0.002f, &output);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, 0.049f, (float)state.nv_activation[0]);
    control_command(&state, direction, 1.0f / 60.0f, true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, 0.048f, (float)state.nv_activation[0]);
    /* A changed thrust must still solve on a step without decay eligibility. */
    state.direction[0] = -0.048f;
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_TRUE(fabs(state.nv_activation[0] - 0.048) > 0.002);
}

static void test_nullspace_zero_vectors_infeasible_and_count_only_reset(void) {
    identity_settings();
    state.settings.nullspace_count = 1;
    state.nv_activation[0] = 0.02;
    state.nv_deadzones[0] = 3;
    command_zero(true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, 0.02f, (float)state.nv_activation[0]);
    TEST_ASSERT_EQUAL_UINT8(3, state.nv_deadzones[0]);
    control_settings_t candidate = state.settings;
    candidate.nullspace[0][0] = 0.001f; /* Deadzone covers all [-.08,.08]. */
    control_apply_settings(&state, &candidate);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, 0.02f, (float)state.nv_activation[0]);
    TEST_ASSERT_EQUAL_UINT8(3, state.nv_deadzones[0]);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)state.nv_activation[0]);
    TEST_ASSERT_EQUAL_UINT8(0, state.nv_deadzones[0]);
    state.nv_activation[0] = 0.03;
    state.nv_deadzones[0] = 7;
    candidate.nullspace_count = 2;
    control_apply_settings(&state, &candidate);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)state.nv_activation[0]);
    TEST_ASSERT_EQUAL_UINT8(0, state.nv_deadzones[0]);
}

static void test_nullspace_disabled_preserves_history_and_does_not_modify_output(void) {
    identity_settings();
    state.settings.nullspace_count = 1;
    state.settings.nullspace[0][0] = 1;
    state.nv_activation[0] = 0.04;
    state.nv_deadzones[0] = 1;
    command_zero(false, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_EQUAL_UINT16(1000, output.motors[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, 0.04f, (float)state.nv_activation[0]);
    TEST_ASSERT_EQUAL_UINT8(1, state.nv_deadzones[0]);
}

/* Fixtures produced by executing the unchanged regulator_b62cbee.txt methods.
 * 2e-6 quaternion tolerance covers binary32 trig/normalization versus SciPy's
 * binary64 rotations; motor quantization is tested separately. */
static void test_python_normal_and_fpv_target_trajectories(void) {
    const float initial[4] = {-0.3796179557f, 0.0935638054f, 0.3464734854f, 0.8526969837f};
    const float commands[4][8] = {
        {0.3f, -0.2f, 0.1f, 0.8f, -0.4f, 0.5f, 0, 0},
        {0, 0, 0, -0.7f, 0.9f, -0.3f, 0, 0},
        {0, 0, 0, 1, 1, 1, 0, 0},
        {0, 0, 0, -0.2f, -0.1f, 0.7f, 0, 0},
    };
    const float expected[2][4][4] = {
        {{-0.3765048282f, 0.1096853831f, 0.3463577372f, 0.8522027628f},
         {-0.3781587853f, 0.0943883956f, 0.3503911540f, 0.8516529828f},
         {-0.3716293411f, 0.1114210051f, 0.3606988153f, 0.8481588042f},
         {-0.3629836215f, 0.1119013167f, 0.3581336406f, 0.8529134078f}},
        {{-0.3791539191f, 0.1064000197f, 0.3375412367f, 0.8549779266f},
         {-0.3775090487f, 0.0975951297f, 0.3492676413f, 0.8520412100f},
         {-0.3714963798f, 0.1203033121f, 0.3487404357f, 0.8519962801f},
         {-0.3625593511f, 0.1201947658f, 0.3481744705f, 0.8560832164f}},
    };
    for (unsigned fpv = 0; fpv < 2; ++fpv) {
        identity_settings();
        state.settings.fpv_mode = fpv != 0;
        state.settings.roll.rate = 90;
        state.settings.pitch.rate = 120;
        state.settings.yaw.rate = 60;
        command_zero(true, false);
        TEST_ASSERT_TRUE(control_set_attitude(&state, initial));
        for (unsigned i = 0; i < 4; ++i) {
            control_command(&state, commands[i], 1.0f / 60.0f, true, false);
            for (unsigned j = 0; j < 4; ++j) {
                TEST_ASSERT_FLOAT_WITHIN(2.0e-6f, expected[fpv][i][j], state.desired_q[j]);
            }
            control_step(&state, &stationary, 0.002f, &output);
        }
    }
}

static void test_python_mahony_noncommuting_trajectory(void) {
    identity_settings();
    const float initial[4] = {-0.3796179557f, 0.0935638054f, 0.3464734854f, 0.8526969837f};
    memcpy(state.current_q, initial, sizeof(initial));
    const control_sample_t sample = {.accel = {0.7f, -1.2f, -9.3f}, .gyro = {0.13f, -0.27f, 0.41f}};
    for (unsigned i = 0; i < 50; ++i) {
        /* .01 is inside both dt clamps, so the baseline runs unchanged. */
        control_step(&state, &sample, 0.01f, &output);
    }
    const float expected_q[4] = {-0.1210919992f, 0.0636524149f, 0.4831978601f, 0.8647571485f};
    const float expected_integral[3] = {0.011452116f, -0.0066447644f, 0.0017193764f};
    for (unsigned i = 0; i < 4; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(2.0e-6f, expected_q[i], state.current_q[i]);
    }
    for (unsigned i = 0; i < 3; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(2.0e-7f, expected_integral[i], state.ahrs_integral[i]);
    }
}

static void test_normal_pitch_limits_and_fpv_no_pitch_limit(void) {
    for (unsigned sign = 0; sign < 2; ++sign) {
        identity_settings();
        state.settings.pitch.rate = 600;
        float direction[8] = {0};
        direction[3] = sign == 0 ? 1.0f : -1.0f;
        control_command(&state, direction, 1.0f / 6.0f, true, false);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, direction[3] * sinf(40.0f * TEST_RAD_PER_DEG),
                                 state.desired_q[1]);
        TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, cosf(40.0f * TEST_RAD_PER_DEG), state.desired_q[3]);
    }
    identity_settings();
    state.settings.pitch.rate = 600;
    state.settings.fpv_mode = true;
    const float direction[8] = {0, 0, 0, 1, 0, 0, 0, 0};
    control_command(&state, direction, 1.0f / 6.0f, true, false);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, sinf(50.0f * TEST_RAD_PER_DEG), state.desired_q[1]);
}

static void test_python_gimbal_and_near_vertical_target_clamping(void) {
    const float targets[4][4] = {
        {-0.4304593205f, 0.5609855056f, 0.4304593205f, 0.5609855056f},
        {-0.0308435652f, -0.7064337730f, -0.0308435652f, 0.7064337730f},
        {-0.4304620326f, 0.5609238744f, 0.4304566383f, 0.5610471964f},
        {-0.0308811292f, -0.7063848376f, -0.0308059994f, 0.7064827085f},
    };
    const float expected[4][4] = {
        {-0.3913043051f, 0.5099576963f, 0.4663383114f, 0.6077439166f},
        {-0.0280380023f, -0.6421758183f, -0.0334143900f, 0.7653153385f},
        {-0.4315144129f, 0.4972812650f, 0.4261281936f, 0.6204203555f},
        {-0.0682529011f, -0.6548599150f, 0.0068005107f, 0.7526312419f},
    };
    for (unsigned i = 0; i < 4; ++i) {
        identity_settings();
        command_zero(true, false);
        TEST_ASSERT_TRUE(control_set_attitude(&state, targets[i]));
        command_zero(true, false);
        for (unsigned j = 0; j < 4; ++j) {
            /* Near 90deg, Euler yaw/roll conditioning amplifies float rounding. */
            TEST_ASSERT_FLOAT_WITHIN(i < 2 ? 2.0e-6f : 2.0e-4f, expected[i][j], state.desired_q[j]);
        }
    }
}

static void test_nullspace_exact_crossing_and_distance_tie_chooses_first_interval(void) {
    identity_settings();
    state.settings.nullspace_count = 1;
    state.settings.nullspace[0][0] = 1;
    state.settings.nullspace[0][1] = 1;
    /* The lower interval has mask0, upper mask3: both one crossing from mask1,
     * and both equally distant from zero. Python chooses sorted first (lower). */
    state.nv_deadzones[0] = 1;
    command_zero(true, false);
    control_step(&state, &stationary, 0.002f, &output);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-9f, -0.003f, (float)state.nv_activation[0]);
    TEST_ASSERT_EQUAL_UINT8(0, state.nv_deadzones[0]);
}

static void test_eight_dense_nullspace_vectors_remain_bounded(void) {
    identity_settings();
    state.settings.nullspace_count = 8;
    state.settings.roll = (control_axis_t){1, 0.1f, 0.2f, 90};
    state.settings.pitch = state.settings.roll;
    state.settings.yaw = state.settings.roll;
    state.settings.depth = (control_axis_t){1, 0.1f, 0.2f, 1};
    for (unsigned i = 0; i < 8; ++i) {
        for (unsigned j = 0; j < 8; ++j) {
            state.settings.nullspace[i][j] = (float)((int)((i + j) % 5) - 2) / 3.0f;
            state.settings.allocation[i][j] = (float)((int)((i * 3 + j) % 7) - 3) / 4.0f;
        }
    }
    TEST_ASSERT_TRUE(control_validate_settings(&state.settings));
    const control_sample_t sample = {.accel = {0.7f, -1.2f, -9.3f}, .gyro = {0.13f, -0.27f, 0.41f}};
    for (unsigned step = 0; step < 1000; ++step) {
        if (step % 8 == 0) {
            float direction[8];
            for (unsigned i = 0; i < 8; ++i) {
                direction[i] = (float)((int)((step + i) % 17) - 8) / 8.0f;
            }
            control_command(&state, direction, 1.0f / 60.0f, true, true);
        }
        control_step(&state, &sample, 0.002f, &output);
        for (unsigned i = 0; i < 8; ++i) {
            TEST_ASSERT_TRUE(output.motors[i] <= 2000);
            TEST_ASSERT_TRUE(isfinite(state.nv_activation[i]));
            TEST_ASSERT_TRUE(fabs(state.nv_activation[i]) <= 0.08);
        }
        for (unsigned i = 0; i < 4; ++i) {
            TEST_ASSERT_TRUE(isfinite(output.current_q[i]));
            TEST_ASSERT_TRUE(isfinite(output.desired_q[i]));
        }
        TEST_ASSERT_TRUE(output.work_percent <= 100);
    }
}

static void test_observe_preserves_control_state_while_advancing_ahrs(void) {
    identity_settings();
    state.settings.depth.ki = 1;
    control_set_depth(&state, 2);
    command_zero(true, true);
    state.attitude_integral[0] = 0.25f;
    state.nv_activation[0] = 0.04;
    state.nv_deadzones[0] = 3;
    float depth_integral = state.depth_integral;
    float target[4];
    memcpy(target, state.desired_q, sizeof(target));
    const control_sample_t sample = {.accel = {0}, .gyro = {1, 0, 0}};
    for (unsigned i = 0; i < 500; ++i) {
        control_observe(&state, &sample, 0.002f, &output);
    }
    TEST_ASSERT_FLOAT_WITHIN(5.0e-6f, sinf(0.5f), output.current_q[0]);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(target, state.desired_q, 4);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, state.attitude_integral[0]);
    TEST_ASSERT_EQUAL_FLOAT(depth_integral, state.depth_integral);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-8f, 0.04f, (float)state.nv_activation[0]);
    TEST_ASSERT_EQUAL_UINT8(3, state.nv_deadzones[0]);
    TEST_ASSERT_TRUE(state.nullspace_decay_pending);
    TEST_ASSERT_TRUE(state.stabilization);
    TEST_ASSERT_TRUE(state.depth_hold);
    for (unsigned i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1000, output.motors[i]);
    }
    TEST_ASSERT_EQUAL_UINT8(0, output.work_percent);
}

static void test_new_session_clears_authority_and_preserves_estimator(void) {
    identity_settings();
    state.settings.nullspace_count = 1;
    state.settings.nullspace[0][0] = 1;
    state.settings.depth.ki = 1;
    control_pressure(&state, 3, 0.2f);
    control_set_depth(&state, 5);
    const float direction[8] = {1, -1, 0.5f, -0.5f, 0.2f, -0.2f, 1, -1};
    control_command(&state, direction, 1.0f / 60.0f, true, true);
    const float target[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    TEST_ASSERT_TRUE(control_set_attitude(&state, target));
    const control_sample_t sample = {.accel = {0.7f, -1.2f, -9.3f}, .gyro = {0.13f, -0.27f, 0.41f}};
    control_step(&state, &sample, 0.002f, &output);
    state.attitude_integral[0] = 0.25f;
    state.nv_activation[0] = 0.04;
    state.nv_deadzones[0] = 3;
    state.nullspace_decay_pending = true;
    control_state_t uninterrupted = state;
    control_settings_t fresh_settings = state.settings;

    control_new_session(&state);

    TEST_ASSERT_EQUAL_FLOAT_ARRAY(uninterrupted.current_q, state.current_q, 4);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(uninterrupted.ahrs_integral, state.ahrs_integral, 3);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(uninterrupted.gyro, state.gyro, 3);
    TEST_ASSERT_FALSE(state.stabilization);
    TEST_ASSERT_FALSE(state.depth_hold);
    TEST_ASSERT_FALSE(state.has_pending_depth);
    TEST_ASSERT_FALSE(state.nullspace_decay_pending);
    TEST_ASSERT_EQUAL_FLOAT(0, state.depth);
    TEST_ASSERT_EQUAL_FLOAT(0, state.depth_change);
    TEST_ASSERT_EQUAL_FLOAT(0, state.desired_depth);
    TEST_ASSERT_EQUAL_FLOAT(0, state.pending_depth);
    TEST_ASSERT_EQUAL_FLOAT(0, state.depth_integral);
    TEST_ASSERT_EQUAL_FLOAT(0, state.depth_actuation);
    const float identity[4] = {0, 0, 0, 1};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(identity, state.desired_q, 4);
    for (unsigned i = 0; i < 3; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(0, state.attitude_integral[i]);
        TEST_ASSERT_EQUAL_FLOAT(0, state.settings.power[i]);
    }
    for (unsigned i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(0, state.direction[i]);
        TEST_ASSERT_TRUE(state.nv_activation[i] == 0.0);
        TEST_ASSERT_EQUAL_UINT8(0, state.nv_deadzones[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(0, state.settings.nullspace_count);
    TEST_ASSERT_TRUE(control_validate_settings(&state.settings));

    control_output_t uninterrupted_output;
    control_observe(&uninterrupted, &sample, 0.002f, &uninterrupted_output);
    control_observe(&state, &sample, 0.002f, &output);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(uninterrupted_output.current_q, output.current_q, 4);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(uninterrupted.ahrs_integral, state.ahrs_integral, 3);
    /* Settings alone cannot revive old input, mode edges, or nullspace output. */
    control_apply_settings(&state, &fresh_settings);
    control_step(&state, &sample, 0.002f, &output);
    for (unsigned i = 0; i < 8; ++i) {
        TEST_ASSERT_EQUAL_UINT16(1000, output.motors[i]);
    }
    TEST_ASSERT_EQUAL_UINT8(0, output.work_percent);
}

static void test_new_session_next_enable_levels_at_measured_yaw(void) {
    identity_settings();
    const float measured[4] = {-0.3796179557f, 0.0935638054f, 0.3464734854f, 0.8526969837f};
    memcpy(state.current_q, measured, sizeof(measured));
    command_zero(true, true);
    control_set_depth(&state, 7);
    const float stale_target[4] = {1, 0, 0, 0};
    TEST_ASSERT_TRUE(control_set_attitude(&state, stale_target));
    control_settings_t fresh_settings = state.settings;
    control_new_session(&state);
    control_apply_settings(&state, &fresh_settings);
    control_pressure(&state, 4, 0);
    command_zero(true, true);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-6f, 0, state.desired_q[0]);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-6f, 0, state.desired_q[1]);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-6f, sinf(17.5f * TEST_RAD_PER_DEG), state.desired_q[2]);
    TEST_ASSERT_FLOAT_WITHIN(2.0e-6f, cosf(17.5f * TEST_RAD_PER_DEG), state.desired_q[3]);
    TEST_ASSERT_EQUAL_FLOAT(4, state.desired_depth);
    TEST_ASSERT_FALSE(state.has_pending_depth);
}

static void test_finite_extreme_rates_keep_quaternion_state_normalized(void) {
    const float rates[] = {1.0e22f, FLT_MAX, -FLT_MAX};
    for (unsigned fpv = 0; fpv < 2; ++fpv) {
        for (unsigned rate = 0; rate < 3; ++rate) {
            identity_settings();
            control_settings_t settings = state.settings;
            settings.fpv_mode = fpv != 0;
            settings.roll.rate = rates[rate];
            settings.pitch.rate = rates[rate];
            settings.yaw.rate = rates[rate];
            TEST_ASSERT_TRUE(control_validate_settings(&settings));
            control_apply_settings(&state, &settings);
            command_zero(true, false);
            const float initial[4] = {-0.3796179557f, 0.0935638054f, 0.3464734854f, 0.8526969837f};
            TEST_ASSERT_TRUE(control_set_attitude(&state, initial));
            for (unsigned mask = 0; mask < 8; ++mask) {
                const float direction[8] = {0,
                                            0,
                                            0,
                                            (mask & 1u) != 0 ? 1.0f : -1.0f,
                                            (mask & 2u) != 0 ? 1.0f : -1.0f,
                                            (mask & 4u) != 0 ? 1.0f : -1.0f,
                                            0,
                                            0};
                control_command(&state, direction, 1.0f / 6.0f, true, false);
                float norm_squared = 0;
                for (unsigned i = 0; i < 4; ++i) {
                    TEST_ASSERT_TRUE(isfinite(state.desired_q[i]));
                    norm_squared += state.desired_q[i] * state.desired_q[i];
                }
                TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 1.0f, norm_squared);
                control_step(&state, &stationary, 0.002f, &output);
                for (unsigned i = 0; i < 4; ++i) {
                    TEST_ASSERT_TRUE(isfinite(output.current_q[i]));
                    TEST_ASSERT_TRUE(isfinite(output.desired_q[i]));
                }
                for (unsigned i = 0; i < 3; ++i) {
                    TEST_ASSERT_TRUE(isfinite(state.attitude_integral[i]));
                }
            }
        }
    }
}

void test_controller(void) {
    RUN_TEST(test_init_and_settings_validation);
    RUN_TEST(test_all_axis_and_coefficient_floats_validated);
    RUN_TEST(test_gyro_exponential_uses_500hz_dt_and_normalizes);
    RUN_TEST(test_step_dt_clamp_is_separate_from_command_clamp);
    RUN_TEST(test_mahony_ned_cross_order_integral_and_gyro_only_fallback);
    RUN_TEST(test_gyro_rejection_preserves_original_pid_derivative);
    RUN_TEST(test_absolute_target_normalization_rejection_and_enable_edge);
    RUN_TEST(test_pid_shortest_rotation_threshold_clips_and_no_antiwindup);
    RUN_TEST(test_targets_and_depth_pid_advance_only_on_commands);
    RUN_TEST(test_depth_pending_targets_disable_edges_and_negative_integration);
    RUN_TEST(test_depth_integral_relaxation_and_clip);
    RUN_TEST(test_allocation_power_work_reorder_spin_and_truncation);
    RUN_TEST(test_user_and_regulator_limits_are_independent);
    RUN_TEST(test_movement_transform_coefficients_zero_ratios_and_yaw_removal);
    RUN_TEST(test_nullspace_initial_crossing_choice_and_sequential_order);
    RUN_TEST(test_nullspace_decay_only_once_per_command);
    RUN_TEST(test_nullspace_zero_vectors_infeasible_and_count_only_reset);
    RUN_TEST(test_nullspace_disabled_preserves_history_and_does_not_modify_output);
    RUN_TEST(test_python_normal_and_fpv_target_trajectories);
    RUN_TEST(test_python_mahony_noncommuting_trajectory);
    RUN_TEST(test_normal_pitch_limits_and_fpv_no_pitch_limit);
    RUN_TEST(test_python_gimbal_and_near_vertical_target_clamping);
    RUN_TEST(test_nullspace_exact_crossing_and_distance_tie_chooses_first_interval);
    RUN_TEST(test_eight_dense_nullspace_vectors_remain_bounded);
    RUN_TEST(test_observe_preserves_control_state_while_advancing_ahrs);
    RUN_TEST(test_new_session_clears_authority_and_preserves_estimator);
    RUN_TEST(test_new_session_next_enable_levels_at_measured_yaw);
    RUN_TEST(test_finite_extreme_rates_keep_quaternion_state_normalized);
}

#ifdef CONTROL_TEST_STANDALONE
void setUp(void) {}
void tearDown(void) {}
int main(void) {
    UNITY_BEGIN();
    test_controller();
    return UNITY_END();
}
#endif
