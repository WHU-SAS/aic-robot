#ifndef AIC_STRAIGHT_CONTROL_H
#define AIC_STRAIGHT_CONTROL_H
#include "remote_control.h"

/* First-run parameters: nominal 65mm wheel, GMR500 * 28 * 2 edges.
 * Coast time is an initial estimate from the 400/600ms ground traces,
 * not a fitted or physically calibrated stopping model. */
#define STRAIGHT_MM_PER_COUNT (3.141592653589793f * 65.0f / 28000.0f)
#define STRAIGHT_CRUISE_MM_S 80.0f
#define STRAIGHT_FINISH_MM_S 25.0f
#define STRAIGHT_ACCEL_MM_S2 150.0f
#define STRAIGHT_COAST_S 0.12f
/* Stronger straight-only accumulated wheel-travel correction. */
#define STRAIGHT_SYNC_GAIN 10.0f
#define STRAIGHT_SPEED_KP 5.0f
#define STRAIGHT_SPEED_KI 4.0f

typedef struct {
    uint32_t start_a, start_b, previous_a, previous_b;
    float speed_left, speed_right, reference_speed;
    float integral_left, integral_right;
} StraightControl;

static inline float straight_clip(float v, float low, float high)
{
    return v < low ? low : (v > high ? high : v);
}

static inline void straight_begin(StraightControl *s, uint32_t a, uint32_t b)
{
    *s = (StraightControl){.start_a=a, .start_b=b, .previous_a=a, .previous_b=b};
}

static inline int16_t straight_wheel(float reference, float speed, float *integral,
                                     int16_t previous)
{
    float error = reference - speed;
    float ff = 1000.0f + 3.0f * reference;
    float next_integral = straight_clip(*integral + STRAIGHT_SPEED_KI * error * 0.01f,
                                        -500.0f, 500.0f);
    float output = ff + STRAIGHT_SPEED_KP * error + next_integral;
    /* Anti-windup is part of the speed PI, not an extra fault mechanism. */
    if ((output >= 0 && output <= REMOTE_PWM_TARGET) ||
        (output > REMOTE_PWM_TARGET && error < 0) || (output < 0 && error > 0))
        *integral = next_integral;
    output = ff + STRAIGHT_SPEED_KP * error + *integral;
    return remote_ramp(previous, (int16_t)straight_clip(output, 0, REMOTE_PWM_TARGET));
}

/* Called at 10ms, after the existing remote_step (B21/heartbeat/deadline).
 * A+ and B- are forward. For reverse MOVE, feedback and PWM magnitudes use
 * the requested travel direction. Once output is removed, no correction is issued. */
static inline void straight_step(StraightControl *s, RemoteControl *motor,
                                 uint32_t a, uint32_t b)
{
    if (!motor->running || !motor->distance_mm) return;
    int direction = motor->reverse_move ? -1 : 1;
    float left = (int32_t)(a - s->start_a) * STRAIGHT_MM_PER_COUNT * direction;
    float right = (int32_t)(b - s->start_b) * STRAIGHT_MM_PER_COUNT * -direction;
    float vl = (int32_t)(a - s->previous_a) * STRAIGHT_MM_PER_COUNT * 100.0f * direction;
    float vr = (int32_t)(b - s->previous_b) * STRAIGHT_MM_PER_COUNT * 100.0f * -direction;
    s->previous_a = a; s->previous_b = b;
    s->speed_left += 0.2f * (vl - s->speed_left);
    s->speed_right += 0.2f * (vr - s->speed_right);
    float remaining = motor->distance_mm - (left + right) * 0.5f;
    float speed = (s->speed_left + s->speed_right) * 0.5f;
    float coast = speed * STRAIGHT_COAST_S;
    if (coast < 0.5f) coast = 0.5f;
    if (remaining <= coast) {
        remote_stop(motor, 6); /* Distance cutoff; coast is observed by K230. */
        return;
    }
    float desired = straight_clip(remaining * 2.0f, STRAIGHT_FINISH_MM_S,
                                   STRAIGHT_CRUISE_MM_S);
    s->reference_speed += straight_clip(desired - s->reference_speed,
        -STRAIGHT_ACCEL_MM_S2 * 0.01f, STRAIGHT_ACCEL_MM_S2 * 0.01f);
    float correction = STRAIGHT_SYNC_GAIN * (left - right);
    float ref_left = straight_clip(s->reference_speed - correction, 0, STRAIGHT_CRUISE_MM_S);
    float ref_right = straight_clip(s->reference_speed + correction, 0, STRAIGHT_CRUISE_MM_S);
    motor->left_pwm = (int16_t)(direction * straight_wheel(ref_left, s->speed_left,
        &s->integral_left, (int16_t)(direction * motor->left_pwm)));
    motor->right_pwm = (int16_t)(direction * straight_wheel(ref_right, s->speed_right,
        &s->integral_right, (int16_t)(direction * motor->right_pwm)));
    motor->target_left = motor->left_pwm; motor->target_right = motor->right_pwm;
    uint16_t left_pwm = remote_magnitude(motor->left_pwm), right_pwm = remote_magnitude(motor->right_pwm);
    motor->pwm = left_pwm > right_pwm ? left_pwm : right_pwm;
}
#endif
