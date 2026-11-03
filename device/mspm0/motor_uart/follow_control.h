#ifndef AIC_FOLLOW_CONTROL_H
#define AIC_FOLLOW_CONTROL_H
#include "straight_control.h"

#define FOLLOW_CAP_CRUISE_MM_S 70.0f
#define FOLLOW_CAP_FINISH_MM_S 50.0f
#define FOLLOW_CAP_SLOWDOWN_GAIN_PER_S 2.0f

/* Continuous signed wheel-speed control. Updates preserve PI and ramp state;
 * unequal wheel targets intentionally steer, with no equal-distance correction. */
typedef struct {
    uint32_t previous_a, previous_b;
    uint32_t cap_start_a, cap_start_b;
    float speed_left, speed_right, reference_left, reference_right;
    float integral_left, integral_right;
} FollowControl;

static inline void follow_begin(FollowControl *s, uint32_t a, uint32_t b)
{
    *s = (FollowControl){.previous_a=a, .previous_b=b};
}

static inline bool follow_cap_cutoff(FollowControl *s, RemoteControl *motor,
                                     uint32_t a, uint32_t b, float *remaining)
{
    float left = (int32_t)(a-s->cap_start_a)*STRAIGHT_MM_PER_COUNT;
    float right = -(float)(int32_t)(b-s->cap_start_b)*STRAIGHT_MM_PER_COUNT;
    *remaining = motor->follow_remaining_mm-(left+right)*0.5f;
    float coast = (s->speed_left+s->speed_right)*0.5f*STRAIGHT_COAST_S;
    if (coast < 0) coast = 0;
    if (*remaining <= coast) {
        remote_stop(motor,6);
        return true;
    }
    return false;
}

/* Attach an endpoint to the accepted encoder snapshot. Do not touch PI,
 * filtered speeds, references, PWM, or the existing run/lease clocks. */
static inline void follow_set_limit(FollowControl *s, RemoteControl *motor,
                                    uint32_t a, uint32_t b)
{
    if (!motor->running || !motor->follow_cap_active) return;
    s->cap_start_a = a; s->cap_start_b = b;
    float remaining;
    (void)follow_cap_cutoff(s,motor,a,b,&remaining); /* Zero/inside coast: STOP before ACK. */
}

static inline int16_t follow_wheel(float target, float speed, float *reference,
                                   float *integral, int16_t previous)
{
    float previous_reference = *reference;
    *reference += straight_clip(target - *reference,
        -STRAIGHT_ACCEL_MM_S2 * 0.01f, STRAIGHT_ACCEL_MM_S2 * 0.01f);
    if (*reference == 0) {
        /* The straight PI's fixed breakaway feedforward must not drive zero. */
        *integral = 0;
        return remote_ramp(previous, 0);
    }
    if ((*reference < 0 && previous_reference > 0) ||
        (*reference > 0 && previous_reference < 0)) *integral = 0;
    if ((*reference < 0 && previous > 0) || (*reference > 0 && previous < 0)) {
        /* Finish this wheel's PWM ramp to zero before applying reverse drive.
         * The other wheel retains its independent reference and PI state. */
        *integral = 0;
        return remote_ramp(previous, 0);
    }
    if (*reference < 0) return (int16_t)-straight_wheel(-*reference, -speed,
        integral, (int16_t)-previous);
    return straight_wheel(*reference, speed, integral, previous);
}

static inline void follow_step(FollowControl *s, RemoteControl *motor,
                                uint32_t a, uint32_t b)
{
    if (!motor->running || !motor->follow_mode) return;
    float vl = (int32_t)(a - s->previous_a) * STRAIGHT_MM_PER_COUNT * 100.0f;
    float vr = -(int32_t)(b - s->previous_b) * STRAIGHT_MM_PER_COUNT * 100.0f;
    s->previous_a = a; s->previous_b = b;
    s->speed_left += 0.2f * (vl - s->speed_left);
    s->speed_right += 0.2f * (vr - s->speed_right);
    float left_target = motor->left_speed, right_target = motor->right_speed;
    if (motor->follow_cap_active) {
        float remaining;
        if (follow_cap_cutoff(s,motor,a,b,&remaining)) return;
        /* Existing70/50/2 parking profile limits the common forward part;
         * preserve the requested left/right differential for steering. */
        float base = (left_target+right_target)*0.5f;
        float ceiling = straight_clip(remaining*FOLLOW_CAP_SLOWDOWN_GAIN_PER_S,
            FOLLOW_CAP_FINISH_MM_S,FOLLOW_CAP_CRUISE_MM_S);
        float reduction = base > ceiling ? base-ceiling : 0;
        left_target -= reduction; right_target -= reduction;
    }
    motor->left_pwm = follow_wheel(left_target, s->speed_left,
        &s->reference_left, &s->integral_left, motor->left_pwm);
    motor->right_pwm = follow_wheel(right_target, s->speed_right,
        &s->reference_right, &s->integral_right, motor->right_pwm);
    motor->target_left = motor->left_pwm; motor->target_right = motor->right_pwm;
    uint16_t left = remote_magnitude(motor->left_pwm), right = remote_magnitude(motor->right_pwm);
    motor->pwm = left > right ? left : right;
}
#endif
