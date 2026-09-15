#include "control/controller.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CONTROL_RAD_PER_DEG 0.01745329251994329577f
#define CONTROL_PI 3.14159265358979323846f

typedef struct {
    double lower;
    double upper;
} control_interval_t;

static float clip(float value, float lower, float upper) {
    return fminf(fmaxf(value, lower), upper);
}

static float clamp_dt(float dt, float frequency) {
    return isfinite(dt) ? clip(dt, 0.5f / frequency, 10.0f / frequency) : 1.0f / frequency;
}

static bool finite_vector(const float *vector, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(vector[i])) {
            return false;
        }
    }
    return true;
}

static bool normalize_quaternion(float q[4]) {
    if (!finite_vector(q, 4)) {
        return false;
    }
    float scale = 0.0f;
    for (size_t i = 0; i < 4; ++i) {
        scale = fmaxf(scale, fabsf(q[i]));
    }
    if (scale < 1.0e-12f) {
        return false;
    }
    float norm_squared = 0.0f;
    for (size_t i = 0; i < 4; ++i) {
        q[i] /= scale;
        norm_squared += q[i] * q[i];
    }
    float inverse_norm = 1.0f / sqrtf(norm_squared);
    for (size_t i = 0; i < 4; ++i) {
        q[i] *= inverse_norm;
    }
    return true;
}

/* Alias-safe Hamilton product, [x,y,z,w]. */
static void quaternion_multiply(const float a[4], const float b[4], float result[4]) {
    float q[4] = {
        (a[3] * b[0]) + (a[0] * b[3]) + (a[1] * b[2]) - (a[2] * b[1]),
        (a[3] * b[1]) - (a[0] * b[2]) + (a[1] * b[3]) + (a[2] * b[0]),
        (a[3] * b[2]) + (a[0] * b[1]) - (a[1] * b[0]) + (a[2] * b[3]),
        (a[3] * b[3]) - (a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]),
    };
    /* Never replace an existing rotation with a failed arithmetic result. */
    if (normalize_quaternion(q)) {
        memcpy(result, q, sizeof(q));
    }
}

static bool quaternion_from_rotvec(const float vector[3], float q[4]) {
    float angle =
        sqrtf((vector[0] * vector[0]) + (vector[1] * vector[1]) + (vector[2] * vector[2]));
    if (!isfinite(angle)) {
        /* Retain ordinary rounding, but avoid overflowing squared finite rates. */
        angle = hypotf(hypotf(vector[0], vector[1]), vector[2]);
    }
    if (!isfinite(angle)) {
        return false;
    }
    float scale = angle < 1.0e-3f ? 0.5f - (angle * angle / 48.0f) : sinf(angle * 0.5f) / angle;
    for (size_t i = 0; i < 3; ++i) {
        q[i] = vector[i] * scale;
    }
    q[3] = cosf(angle * 0.5f);
    return finite_vector(q, 4);
}

static void quaternion_to_euler(const float q[4], float *yaw, float *pitch, float *roll) {
    float sin_pitch = 2.0f * (q[3] * q[1] - q[2] * q[0]);
    float yaw_sin = 2.0f * (q[3] * q[2] + q[0] * q[1]);
    float yaw_cos = (q[3] * q[3]) + (q[0] * q[0]) - (q[1] * q[1]) - (q[2] * q[2]);
    float cos_pitch = hypotf(yaw_sin, yaw_cos);
    /* asin(sin_pitch) loses the distinction between near-vertical and gimbal
     * lock in binary32. Homogeneous matrix entries also tolerate norm rounding. */
    *pitch = atan2f(sin_pitch, cos_pitch);
    if (cos_pitch <= 1.0e-7f) {
        /* scipy ZYX convention at gimbal lock sets the third angle to zero. */
        *yaw = 2.0f * atan2f(q[2], q[3]);
        *yaw = remainderf(*yaw, 2.0f * CONTROL_PI);
        *roll = 0.0f;
        return;
    }
    *yaw = atan2f(yaw_sin, yaw_cos);
    float roll_cos = (q[3] * q[3]) - (q[0] * q[0]) - (q[1] * q[1]) + (q[2] * q[2]);
    *roll = atan2f(2.0f * (q[3] * q[0] + q[1] * q[2]), roll_cos);
}

static void quaternion_from_euler(float yaw, float pitch, float roll, float q[4]) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    q[0] = cy * cp * sr - sy * sp * cr;
    q[1] = cy * sp * cr + sy * cp * sr;
    q[2] = sy * cp * cr - cy * sp * sr;
    q[3] = cy * cp * cr + sy * sp * sr;
    (void)normalize_quaternion(q);
}

static void update_ahrs(control_state_t *state, const control_sample_t *sample, float dt) {
    /* AHRS rejects the entire excessive sample; the PID retains its derivative. */
    memcpy(state->gyro, sample->gyro, sizeof(state->gyro));
    float omega[3];
    memcpy(omega, sample->gyro, sizeof(omega));
    for (size_t i = 0; i < 3; ++i) {
        if (fabsf(sample->gyro[i]) > 1080.0f * CONTROL_RAD_PER_DEG) {
            memset(omega, 0, sizeof(omega));
            break;
        }
    }
    /* Health gating belongs to the caller. Do not poison the stored quaternion. */
    if (!finite_vector(omega, 3)) {
        return;
    }
    float norm =
        sqrtf((sample->accel[0] * sample->accel[0]) + (sample->accel[1] * sample->accel[1]) +
              (sample->accel[2] * sample->accel[2]));
    if (isfinite(norm) && norm >= 1.0e-3f) {
        float a[3];
        for (size_t i = 0; i < 3; ++i) {
            a[i] = sample->accel[i] / norm;
        }
        const float *q = state->current_q;
        /* Inverse body-to-world rotation applied to NED up [0,0,-1]. */
        float up[3] = {2.0f * (q[3] * q[1] - q[0] * q[2]), -2.0f * (q[1] * q[2] + q[3] * q[0]),
                       -1.0f + (2.0f * (q[0] * q[0] + q[1] * q[1]))};
        float error[3] = {(a[1] * up[2]) - (a[2] * up[1]), (a[2] * up[0]) - (a[0] * up[2]),
                          (a[0] * up[1]) - (a[1] * up[0])};
        for (size_t i = 0; i < 3; ++i) {
            state->ahrs_integral[i] += error[i] * (0.05f * dt);
            omega[i] += 1.5f * error[i] + state->ahrs_integral[i];
        }
    }
    for (size_t i = 0; i < 3; ++i) {
        omega[i] *= dt;
    }
    float delta[4];
    if (quaternion_from_rotvec(omega, delta)) {
        quaternion_multiply(state->current_q, delta, state->current_q);
    }
}

void control_init(control_state_t *state) {
    memset(state, 0, sizeof(*state));
    state->current_q[3] = 1.0f;
    state->desired_q[3] = 1.0f;
    for (size_t i = 0; i < 8; ++i) {
        state->settings.identifiers[i] = (uint8_t)i;
        state->settings.spin[i] = 1;
    }
    for (size_t i = 0; i < 3; ++i) {
        state->settings.coefficients[i] = 1.0f;
    }
}

void control_new_session(control_state_t *state) {
    float current_q[4];
    float ahrs_integral[3];
    float gyro[3];
    memcpy(current_q, state->current_q, sizeof(current_q));
    memcpy(ahrs_integral, state->ahrs_integral, sizeof(ahrs_integral));
    memcpy(gyro, state->gyro, sizeof(gyro));
    control_init(state);
    memcpy(state->current_q, current_q, sizeof(current_q));
    memcpy(state->ahrs_integral, ahrs_integral, sizeof(ahrs_integral));
    memcpy(state->gyro, gyro, sizeof(gyro));
}

static bool validate_axis(const control_axis_t *axis) {
    return isfinite(axis->kp) && isfinite(axis->ki) && isfinite(axis->kd) && isfinite(axis->rate);
}

bool control_validate_settings(const control_settings_t *settings) {
    if (settings == NULL || !validate_axis(&settings->roll) || !validate_axis(&settings->pitch) ||
        !validate_axis(&settings->yaw) || !validate_axis(&settings->depth) ||
        !finite_vector(settings->coefficients, 3) || settings->nullspace_count > 8) {
        return false;
    }
    for (size_t i = 0; i < 3; ++i) {
        if (!isfinite(settings->power[i]) || settings->power[i] < 0.0f ||
            settings->power[i] > 100.0f) {
            return false;
        }
    }
    for (size_t i = 0; i < 8; ++i) {
        if (settings->identifiers[i] > 7 || (settings->spin[i] != -1 && settings->spin[i] != 1) ||
            !finite_vector(settings->allocation[i], 8) ||
            !finite_vector(settings->nullspace[i], 8)) {
            return false;
        }
        for (size_t j = 0; j < 8; ++j) {
            if (i >= settings->nullspace_count && settings->nullspace[i][j] != 0.0f) {
                return false;
            }
        }
    }
    return true;
}

void control_apply_settings(control_state_t *state, const control_settings_t *settings) {
    if (!control_validate_settings(settings)) {
        return;
    }
    if (state->settings.nullspace_count != settings->nullspace_count) {
        memset(state->nv_activation, 0, sizeof(state->nv_activation));
        memset(state->nv_deadzones, 0, sizeof(state->nv_deadzones));
    }
    state->settings = *settings;
}

void control_pressure(control_state_t *state, float depth, float depth_change) {
    if (isfinite(depth) && isfinite(depth_change)) {
        state->depth = depth;
        state->depth_change = depth_change;
    }
}

bool control_set_attitude(control_state_t *state, const float q[4]) {
    float normalized[4];
    memcpy(normalized, q, sizeof(normalized));
    if (!normalize_quaternion(normalized)) {
        return false;
    }
    memcpy(state->desired_q, normalized, sizeof(normalized));
    return true;
}

void control_set_depth(control_state_t *state, float depth) {
    if (!isfinite(depth)) {
        return;
    }
    state->pending_depth = fmaxf(0.0f, depth);
    state->has_pending_depth = true;
    if (state->depth_hold) {
        state->desired_depth = state->pending_depth;
    }
}

static void update_target(control_state_t *state, float dt) {
    float vector[3] = {state->direction[5] * dt * state->settings.roll.rate * CONTROL_RAD_PER_DEG,
                       state->direction[3] * dt * state->settings.pitch.rate * CONTROL_RAD_PER_DEG,
                       state->direction[4] * dt * state->settings.yaw.rate * CONTROL_RAD_PER_DEG};
    if (!finite_vector(vector, 3)) {
        return;
    }
    float delta[4];
    if (state->settings.fpv_mode) {
        if (quaternion_from_rotvec(vector, delta)) {
            quaternion_multiply(state->desired_q, delta, state->desired_q);
        }
        return;
    }
    float yaw_vector[3] = {0.0f, 0.0f, vector[2]};
    if (!quaternion_from_rotvec(yaw_vector, delta)) {
        return;
    }
    quaternion_multiply(delta, state->desired_q, state->desired_q);
    float yaw;
    float pitch;
    float roll;
    quaternion_to_euler(state->desired_q, &yaw, &pitch, &roll);
    pitch = clip(pitch + vector[1], -80.0f * CONTROL_RAD_PER_DEG, 80.0f * CONTROL_RAD_PER_DEG);
    quaternion_from_euler(yaw, pitch, roll, state->desired_q);
    float roll_vector[3] = {vector[0], 0.0f, 0.0f};
    if (quaternion_from_rotvec(roll_vector, delta)) {
        quaternion_multiply(state->desired_q, delta, state->desired_q);
    }
}

void control_command(control_state_t *state, const float direction[8], float source_dt,
                     bool stabilization, bool depth_hold) {
    float dt = clamp_dt(source_dt, 60.0f);
    if (depth_hold && !state->depth_hold) {
        state->depth_integral = 0.0f;
        state->desired_depth = state->has_pending_depth ? state->pending_depth : state->depth;
    } else if (!depth_hold && state->depth_hold) {
        state->has_pending_depth = false;
    }
    if (stabilization && !state->stabilization) {
        float yaw;
        float pitch;
        float roll;
        quaternion_to_euler(state->current_q, &yaw, &pitch, &roll);
        quaternion_from_euler(yaw, 0.0f, 0.0f, state->desired_q);
        memset(state->attitude_integral, 0, sizeof(state->attitude_integral));
    }
    state->stabilization = stabilization;
    state->depth_hold = depth_hold;
    memcpy(state->direction, direction, sizeof(state->direction));
    state->nullspace_decay_pending = true;
    if (stabilization) {
        update_target(state, dt);
    }
    if (depth_hold) {
        state->desired_depth += direction[2] * state->settings.depth.rate * dt;
        float error = state->desired_depth - state->depth;
        float integral_scale = clip(1.0f - fabsf(direction[2]), 0.0f, 1.0f);
        state->depth_integral =
            clip(state->depth_integral + (error * dt * integral_scale), -3.0f, 3.0f);
        state->depth_actuation = state->settings.depth.kp * error +
                                 state->settings.depth.ki * state->depth_integral -
                                 state->settings.depth.kd * state->depth_change;
    }
}

static void attitude_pid(control_state_t *state, float dt, float actuation[3]) {
    float inverse[4] = {-state->current_q[0], -state->current_q[1], -state->current_q[2],
                        state->current_q[3]};
    float error_q[4] = {0, 0, 0, 1};
    quaternion_multiply(inverse, state->desired_q, error_q);
    /* scipy canonicalizes the pi tie lexicographically as well as w < 0. */
    bool negate = error_q[3] < 0.0f;
    if (error_q[3] == 0.0f) {
        for (size_t i = 0; i < 3; ++i) {
            if (error_q[i] != 0.0f) {
                negate = error_q[i] < 0.0f;
                break;
            }
        }
    }
    if (negate) {
        for (size_t i = 0; i < 4; ++i) {
            error_q[i] = -error_q[i];
        }
    }
    float norm =
        sqrtf((error_q[0] * error_q[0]) + (error_q[1] * error_q[1]) + (error_q[2] * error_q[2]));
    float angle = 2.0f * atan2f(norm, error_q[3]);
    float scale = angle < 1.0e-3f ? 2.0f + (angle * angle / 12.0f) : angle / sinf(angle * 0.5f);
    float input_norm = sqrtf((state->direction[3] * state->direction[3]) +
                             (state->direction[4] * state->direction[4]) +
                             (state->direction[5] * state->direction[5]));
    const control_axis_t *axes[3] = {&state->settings.roll, &state->settings.pitch,
                                     &state->settings.yaw};
    const size_t output_indices[3] = {2, 0, 1};
    for (size_t i = 0; i < 3; ++i) {
        float error = error_q[i] * scale;
        if (!isfinite(error)) {
            error = 0.0f;
        }
        if (input_norm < 0.2f) {
            state->attitude_integral[i] += error * dt;
        }
        state->attitude_integral[i] =
            clip(state->attitude_integral[i], -100.0f * CONTROL_RAD_PER_DEG,
                 100.0f * CONTROL_RAD_PER_DEG);
        actuation[output_indices[i]] =
            (axes[i]->kp * error + axes[i]->ki * state->attitude_integral[i] -
             axes[i]->kd * state->gyro[i]) /
            10.0f;
    }
}

static void transform_movement(const control_state_t *state, const float movement[3],
                               float body[3]) {
    float yaw;
    float pitch;
    float roll;
    quaternion_to_euler(state->current_q, &yaw, &pitch, &roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cr = cosf(roll);
    float sr = sinf(roll);
    float surge = state->settings.coefficients[0];
    float sway = state->settings.coefficients[1];
    float heave = state->settings.coefficients[2];
    float heave_surge = surge != 0.0f ? heave / surge : 0.0f;
    float heave_sway = sway != 0.0f ? heave / sway : 0.0f;
    float surge_heave = heave != 0.0f ? surge / heave : 0.0f;
    float sway_heave = heave != 0.0f ? sway / heave : 0.0f;
    float result[3] = {(cp * movement[0]) - (sp * movement[2] * heave_surge),
                       (sp * sr * movement[0]) + (cr * movement[1]) +
                           (cp * sr * movement[2] * heave_sway),
                       (sp * cr * movement[0] * surge_heave) - (sr * movement[1] * sway_heave) +
                           (cp * cr * movement[2])};
    memcpy(body, result, sizeof(result));
}

static unsigned bit_count(uint8_t bits) {
    unsigned count = 0;
    while (bits != 0) {
        count += bits & 1u;
        bits >>= 1u;
    }
    return count;
}

typedef struct {
    control_interval_t deadzones[8];
    control_interval_t available[9];
    size_t active_count;
    size_t available_count;
} control_nv_intervals_t;

static void insert_interval(control_interval_t intervals[8], size_t count,
                            control_interval_t zone) {
    /* Lexicographic insertion sort matches Python tuple ordering. */
    size_t at = count;
    while (at > 0 &&
           (intervals[at - 1].lower > zone.lower ||
            (intervals[at - 1].lower == zone.lower && intervals[at - 1].upper > zone.upper))) {
        intervals[at] = intervals[at - 1];
        --at;
    }
    intervals[at] = zone;
}

static void calculate_intervals(const float nv[8], const float thrust[8],
                                control_nv_intervals_t *result) {
    control_interval_t forbidden[8];
    size_t forbidden_count = 0;
    result->active_count = 0;
    result->available_count = 0;
    for (size_t i = 0; i < 8; ++i) {
        if (nv[i] == 0.0f) {
            continue;
        }
        float a = -((thrust[i] - 0.003f) / nv[i]);
        float b = -((thrust[i] + 0.003f) / nv[i]);
        control_interval_t zone = {.lower = fminf(a, b), .upper = fmaxf(a, b)};
        result->deadzones[result->active_count++] = zone;
        zone.lower = fmax(zone.lower, -0.08);
        zone.upper = fmin(zone.upper, 0.08);
        if (zone.lower < zone.upper) {
            insert_interval(forbidden, forbidden_count++, zone);
        }
    }
    /* Advancing the cursor merges touching/overlapping forbidden intervals. */
    double cursor = -0.08;
    for (size_t i = 0; i < forbidden_count; ++i) {
        if (cursor < forbidden[i].lower) {
            result->available[result->available_count++] =
                (control_interval_t){cursor, forbidden[i].lower};
        }
        cursor = fmax(cursor, forbidden[i].upper);
    }
    if (cursor < 0.08) {
        result->available[result->available_count++] = (control_interval_t){cursor, 0.08};
    }
}

static control_interval_t choose_interval(const control_nv_intervals_t *intervals, double previous,
                                          uint8_t previous_mask, uint8_t *chosen_mask) {
    unsigned best_crossings = 9;
    double best_distance = INFINITY;
    size_t chosen = 0;
    *chosen_mask = 0;
    for (size_t i = 0; i < intervals->available_count; ++i) {
        control_interval_t interval = intervals->available[i];
        double midpoint = (interval.lower + interval.upper) / 2.0;
        uint8_t mask = 0;
        for (size_t j = 0; j < intervals->active_count; ++j) {
            if (intervals->deadzones[j].upper <= midpoint) {
                mask |= (uint8_t)(1u << j);
            }
        }
        unsigned crossings = bit_count((uint8_t)(mask ^ previous_mask));
        double nearest = fmin(fmax(previous, interval.lower), interval.upper);
        double distance = fabs(previous - nearest);
        if (crossings < best_crossings ||
            (crossings == best_crossings && distance < best_distance)) {
            best_crossings = crossings;
            best_distance = distance;
            chosen = i;
            *chosen_mask = mask;
        }
    }
    return intervals->available[chosen];
}

static void remove_deadzone(control_state_t *state, float thrust[8]) {
    if (!state->stabilization) {
        return;
    }
    for (size_t nv_index = 0; nv_index < state->settings.nullspace_count; ++nv_index) {
        const float *nv = state->settings.nullspace[nv_index];
        control_nv_intervals_t intervals;
        calculate_intervals(nv, thrust, &intervals);
        if (intervals.active_count == 0) {
            continue;
        }
        if (intervals.available_count == 0) {
            state->nv_activation[nv_index] = 0.0;
            state->nv_deadzones[nv_index] = 0;
            continue;
        }
        double previous = state->nv_activation[nv_index];
        uint8_t chosen_mask = 0;
        control_interval_t interval =
            choose_interval(&intervals, previous, state->nv_deadzones[nv_index], &chosen_mask);
        double activation = previous;
        if (interval.lower <= previous && previous <= interval.upper &&
            state->nullspace_decay_pending) {
            if (activation > 0.0) {
                activation = fmax(activation - 0.001, 0.0);
            } else if (activation < 0.0) {
                activation = fmin(activation + 0.001, 0.0);
            }
        }
        activation = fmin(fmax(activation, interval.lower), interval.upper);
        for (size_t i = 0; i < 8; ++i) {
            thrust[i] += nv[i] * (float)activation;
        }
        state->nv_activation[nv_index] = activation;
        state->nv_deadzones[nv_index] = chosen_mask;
    }
}

static void output_attitude(const control_state_t *state, control_output_t *output) {
    memcpy(output->current_q, state->current_q, sizeof(output->current_q));
    memcpy(output->desired_q, state->desired_q, sizeof(output->desired_q));
    output->desired_depth = state->depth;
    if (state->depth_hold) {
        output->desired_depth = state->desired_depth;
    } else if (state->has_pending_depth) {
        output->desired_depth = state->pending_depth;
    }
}

void control_observe(control_state_t *state, const control_sample_t *sample, float dt,
                     control_output_t *output) {
    update_ahrs(state, sample, clamp_dt(dt, 500.0f));
    output_attitude(state, output);
    for (size_t i = 0; i < 8; ++i) {
        output->motors[i] = 1000;
    }
    output->work_percent = 0;
}

void control_step(control_state_t *state, const control_sample_t *sample, float dt,
                  control_output_t *output) {
    dt = clamp_dt(dt, 500.0f);
    update_ahrs(state, sample, dt);
    float user[8];
    memcpy(user, state->direction, sizeof(user));
    float regulator[8] = {0};
    if (state->depth_hold) {
        float depth_vector[3] = {0.0f, 0.0f, state->depth_actuation};
        transform_movement(state, depth_vector, regulator);
        user[2] = 0.0f;
        transform_movement(state, user, user);
    }
    if (state->stabilization) {
        attitude_pid(state, dt, &regulator[3]);
        memset(&user[3], 0, 3 * sizeof(float));
    }
    float unlimited[8];
    float limited[8];
    float regulator_limit = state->settings.power[2] / 100.0f;
    for (size_t i = 0; i < 8; ++i) {
        unlimited[i] = user[i] + regulator[i];
        limited[i] = user[i] * (state->settings.power[i < 6 ? 0 : 1] / 100.0f) +
                     clip(regulator[i], -regulator_limit, regulator_limit);
    }
    float thrust[8] = {0};
    double work = 0.0;
    for (size_t i = 0; i < 8; ++i) {
        float work_thrust = 0.0f;
        for (size_t j = 0; j < 8; ++j) {
            thrust[i] += state->settings.allocation[i][j] * limited[j];
            work_thrust += state->settings.allocation[i][j] * unlimited[j];
        }
        work += fabsf(clip(work_thrust, -1.0f, 1.0f));
    }
    output->work_percent = (uint8_t)fmin(100.0, work * 100.0 / 8.0);
    remove_deadzone(state, thrust);
    state->nullspace_decay_pending = false;
    for (size_t i = 0; i < 8; ++i) {
        float value = thrust[state->settings.identifiers[i]] * state->settings.spin[i];
        value = isfinite(value) ? clip(value, -1.0f, 1.0f) : 0.0f;
        output->motors[i] = (uint16_t)(1000.0f + (1000.0f * value));
    }
    output_attitude(state, output);
}
