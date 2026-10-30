#ifndef AIC_TURN_CONTROL_H
#define AIC_TURN_CONTROL_H
#include "straight_control.h"

/* Nominal geometry, not a measured physical yaw calibration. Reuse the
 * straight controller's wheel-speed PI, acceleration and one-shot cutoff. */
#define TURN_TRACK_MM 119.2f
#define TURN_MM_PER_DEG (TURN_TRACK_MM * 3.141592653589793f / 180.0f)
#define TURN_CRUISE_MM_S 40.0f
#define TURN_FINISH_MM_S 20.0f
#define TURN_START_MM 1.0f
/* Trial turn-only coast estimate; ground runs must confirm final angle. */
#define TURN_COAST_S 0.03f

typedef struct {
    StraightControl wheels;
    bool started;
} TurnControl;

static inline void turn_begin(TurnControl *s, uint32_t a, uint32_t b)
{
    straight_begin(&s->wheels, a, b);
    s->started = false;
}

static inline void turn_step(TurnControl *turn, RemoteControl *motor,
                             uint32_t a, uint32_t b)
{
    if (!motor->running || !motor->angle_deg) return;
    StraightControl *s = &turn->wheels;
    float left = (int32_t)(a - s->start_a) * STRAIGHT_MM_PER_COUNT;
    float right = -(int32_t)(b - s->start_b) * STRAIGHT_MM_PER_COUNT;
    float vl = (int32_t)(a - s->previous_a) * STRAIGHT_MM_PER_COUNT * 100.0f;
    float vr = -(int32_t)(b - s->previous_b) * STRAIGHT_MM_PER_COUNT * 100.0f;
    s->previous_a = a; s->previous_b = b;
    s->speed_left += 0.2f * (vl - s->speed_left);
    s->speed_right += 0.2f * (vr - s->speed_right);
    bool turn_left = motor->angle_deg > 0;
    float progress = turn_left ? right - left : left - right;
    if (progress >= TURN_START_MM) turn->started = true;
    float speed = turn_left ? s->speed_right - s->speed_left : s->speed_left - s->speed_right;
    float remaining = remote_magnitude(motor->angle_deg) * TURN_MM_PER_DEG - progress;
    float coast = speed * TURN_COAST_S;
    if (coast < 0.5f) coast = 0.5f;
    if (remaining <= coast) {
        remote_stop(motor, 7);
        return;
    }
    float desired = straight_clip(remaining * 2.0f, TURN_FINISH_MM_S, TURN_CRUISE_MM_S);
    s->reference_speed += straight_clip(desired - s->reference_speed,
        -STRAIGHT_ACCEL_MM_S2 * 0.01f, STRAIGHT_ACCEL_MM_S2 * 0.01f);
    /* One wheel is powered; the other may roll, and its encoder still counts.
     * Compensate that rolling in the moving wheel's speed reference. */
    float passive_speed = turn_left ? s->speed_left : s->speed_right;
    float reference = straight_clip(s->reference_speed + passive_speed, 0, TURN_CRUISE_MM_S);
    /* Small goals must not suppress the torque needed to start. Use the
     * existing ramp/20% ceiling until forward travel, then hand over once
     * to the speed PI. Never return to the startup phase in the same run. */
    int16_t output = turn->started ? straight_wheel(reference,
        turn_left ? s->speed_right : s->speed_left,
        turn_left ? &s->integral_right : &s->integral_left,
        turn_left ? motor->right_pwm : motor->left_pwm) :
        remote_ramp(turn_left ? motor->right_pwm : motor->left_pwm, REMOTE_PWM_TARGET);
    motor->left_pwm = turn_left ? 0 : output;
    motor->right_pwm = turn_left ? output : 0;
    motor->target_left = motor->left_pwm; motor->target_right = motor->right_pwm;
    motor->pwm = motor->left_pwm > motor->right_pwm ? motor->left_pwm : motor->right_pwm;
}
#endif
