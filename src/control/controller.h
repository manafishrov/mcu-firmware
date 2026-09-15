#ifndef CONTROL_CONTROLLER_H
#define CONTROL_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float rate;
} control_axis_t;

typedef struct control_settings {
    control_axis_t roll;
    control_axis_t pitch;
    control_axis_t yaw;
    control_axis_t depth;
    bool fpv_mode;
    float power[3];        /* Thrusters, actions, regulator: percentages. */
    float coefficients[3]; /* Surge, sway, heave. */
    float allocation[8][8];
    uint8_t identifiers[8];
    int8_t spin[8];
    uint32_t nullspace_count;
    float nullspace[8][8];
} control_settings_t;

typedef struct {
    float accel[3]; /* m/s^2, body NED axes. */
    float gyro[3];  /* rad/s, body NED axes. */
} control_sample_t;

typedef struct {
    float current_q[4]; /* Body-to-world [x,y,z,w]. */
    float desired_q[4];
    float desired_depth;
    uint16_t motors[8];
    uint8_t work_percent;
} control_output_t;

/* Core1-owned, allocation-free storage. Only the functions below mutate it. */
typedef struct {
    control_settings_t settings;
    float current_q[4];
    float desired_q[4];
    float ahrs_integral[3];
    float attitude_integral[3];
    float gyro[3]; /* Original derivative sample, before AHRS rejection. */
    float direction[8];
    float depth;
    float depth_change;
    float desired_depth;
    float pending_depth;
    float depth_integral;
    float depth_actuation;
    double nv_activation[8]; /* Python interval/history arithmetic uses binary64. */
    uint8_t nv_deadzones[8]; /* Bits index active entries, not motor channels. */
    bool stabilization;
    bool depth_hold;
    bool has_pending_depth;
    bool nullspace_decay_pending;
} control_state_t;

void control_init(control_state_t *state);
/* Reset to neutral defaults except AHRS quaternion/integral and latest gyro.
 * Caller must require fresh settings, pressure health and command authority. */
void control_new_session(control_state_t *state);
bool control_validate_settings(const control_settings_t *settings);
void control_apply_settings(control_state_t *state, const control_settings_t *settings);
void control_pressure(control_state_t *state, float depth, float depth_change);
void control_command(control_state_t *state, const float direction[8], float source_dt,
                     bool stabilization, bool depth_hold);
bool control_set_attitude(control_state_t *state, const float q[4]);
void control_set_depth(control_state_t *state, float depth);
void control_step(control_state_t *state, const control_sample_t *sample, float dt,
                  control_output_t *output);
/* AHRS-only while authority is inhibited; preserves targets/PID/nullspace history. */
void control_observe(control_state_t *state, const control_sample_t *sample, float dt,
                     control_output_t *output);

#endif
