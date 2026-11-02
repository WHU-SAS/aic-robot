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
/* Millimetre corrections have no long cruise phase. These are initial
 * control candidates, not a physical distance calibration. */
#define STRAIGHT_SHORT_LIMIT_MM 30U
#define STRAIGHT_SHORT_SPEED_MM_S 15.0f
#define STRAIGHT_SHORT_COAST_S 0.08f
/* Stronger straight-only accumulated wheel-travel correction. */
#define STRAIGHT_SYNC_GAIN 10.0f
#define STRAIGHT_SPEED_KP 5.0f
#define STRAIGHT_SPEED_KI 4.0f

typedef struct {
    uint32_t start_a, start_b, previous_a, previous_b;
    float speed_left, speed_right, reference_speed;
    float integral_left, integral_right;
    int16_t short_left_mm, short_right_mm;
    bool short_initialized, left_done, right_done;
    bool short_left_started, short_right_started;
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

/* Signed wheel goals: positive is forward, zero leaves that wheel stopped.
 * ARC callers share this initializer; equal goals implement a short MOVE. */
static inline void straight_short_begin(StraightControl *s, uint32_t a, uint32_t b,
                                        int16_t left_mm, int16_t right_mm)
{
    straight_begin(s, a, b);
    s->short_left_mm = left_mm;
    s->short_right_mm = right_mm;
    s->short_initialized = true;
    s->left_done = !left_mm;
    s->right_done = !right_mm;
}

static inline int16_t straight_short_wheel(float reference, float speed, float travel,
                                          bool *started, float *integral, int16_t previous)
{
    if (!*started) {
        if (travel < 0.1f) return remote_ramp(previous, REMOTE_PWM_TARGET);
        *started = true;
        /* Transfer the observed breakaway output into the existing speed PI
         * without dropping a slow, short-target wheel back below its dead zone. */
        *integral = straight_clip(previous-(1000.0f+3.0f*reference)-
            STRAIGHT_SPEED_KP*(reference-speed), -500.0f, 500.0f);
    }
    return reference > 0 ? straight_wheel(reference, speed, integral, previous) :
        remote_ramp(previous, 0);
}

static inline void straight_short_step(StraightControl *s, RemoteControl *motor,
                                       uint32_t a, uint32_t b, uint8_t reason)
{
    if (!motor->running) return;
    float raw_left = (int32_t)(a - s->start_a) * STRAIGHT_MM_PER_COUNT;
    float raw_right = -(int32_t)(b - s->start_b) * STRAIGHT_MM_PER_COUNT;
    float vl = (int32_t)(a - s->previous_a) * STRAIGHT_MM_PER_COUNT * 100.0f;
    float vr = -(int32_t)(b - s->previous_b) * STRAIGHT_MM_PER_COUNT * 100.0f;
    s->previous_a = a; s->previous_b = b;
    s->speed_left += 0.2f * (vl - s->speed_left);
    s->speed_right += 0.2f * (vr - s->speed_right);
    int dl = s->short_left_mm < 0 ? -1 : 1, dr = s->short_right_mm < 0 ? -1 : 1;
    float gl = remote_magnitude(s->short_left_mm), gr = remote_magnitude(s->short_right_mm);
    float longest = gl > gr ? gl : gr;
    float left = raw_left*dl, right = raw_right*dr;
    float cl = s->speed_left*dl*STRAIGHT_SHORT_COAST_S;
    float cr = s->speed_right*dr*STRAIGHT_SHORT_COAST_S;
    if (cl < 0.1f) cl = 0.1f;
    if (cr < 0.1f) cr = 0.1f;
    if (gl-left <= cl) s->left_done = true;
    if (gr-right <= cr) s->right_done = true;
    if (s->left_done && s->right_done) { remote_stop(motor, reason); return; }
    /* A cutoff wheel is finished for synchronization too; its predicted
     * coast must not hold the remaining wheel short of its own target. */
    float pl = gl ? (s->left_done ? longest : left*longest/gl) : 0;
    float pr = gr ? (s->right_done ? longest : right*longest/gr) : 0;
    /* Larger ARC goals share the existing cruise/acceleration limits. Use
     * the dominant wheel's remaining travel so 100mm fits the 5s deadline,
     * then finish with the same low-speed, per-wheel cutoff as tiny goals. */
    float limit = longest > STRAIGHT_SHORT_LIMIT_MM ? STRAIGHT_CRUISE_MM_S : STRAIGHT_SHORT_SPEED_MM_S;
    float remaining = gl >= gr ? gl-left : gr-right;
    float desired = straight_clip(remaining*2.0f, STRAIGHT_SHORT_SPEED_MM_S, limit);
    s->reference_speed += straight_clip(desired-s->reference_speed,
        -STRAIGHT_ACCEL_MM_S2*0.01f, STRAIGHT_ACCEL_MM_S2*0.01f);
    float sync = gl && gr ? STRAIGHT_SYNC_GAIN*(pl-pr) : 0;
    /* On long arcs synchronization slows the leading wheel instead of
     * accelerating its mate above the braking envelope near the endpoint. */
    float sync_limit = longest > STRAIGHT_SHORT_LIMIT_MM ? s->reference_speed : limit;
    float rl = gl ? straight_clip(s->reference_speed-sync,0,sync_limit)*gl/longest : 0;
    float rr = gr ? straight_clip(s->reference_speed+sync,0,sync_limit)*gr/longest : 0;
    motor->left_pwm = s->left_done ? 0 : (int16_t)(dl*straight_short_wheel(rl,
        s->speed_left*dl, left, &s->short_left_started, &s->integral_left, (int16_t)(motor->left_pwm*dl)));
    motor->right_pwm = s->right_done ? 0 : (int16_t)(dr*straight_short_wheel(rr,
        s->speed_right*dr, right, &s->short_right_started, &s->integral_right, (int16_t)(motor->right_pwm*dr)));
    motor->target_left = motor->left_pwm; motor->target_right = motor->right_pwm;
    uint16_t left_pwm=remote_magnitude(motor->left_pwm), right_pwm=remote_magnitude(motor->right_pwm);
    motor->pwm = left_pwm > right_pwm ? left_pwm : right_pwm;
}

/* Called at 10ms, after the existing remote_step (B21/heartbeat/deadline).
 * A+ and B- are forward. For reverse MOVE, feedback and PWM magnitudes use
 * the requested travel direction. Once output is removed, no correction is issued. */
static inline void straight_step(StraightControl *s, RemoteControl *motor,
                                 uint32_t a, uint32_t b)
{
    if (!motor->running || !motor->distance_mm) return;
    if (motor->distance_mm <= STRAIGHT_SHORT_LIMIT_MM) {
        if (!s->short_initialized) {
            int16_t goal=(int16_t)(motor->reverse_move ? -(int)motor->distance_mm : motor->distance_mm);
            straight_short_begin(s,s->start_a,s->start_b,goal,goal);
        }
        straight_short_step(s,motor,a,b,6);
        return;
    }
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
