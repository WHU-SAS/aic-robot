#ifndef AIC_REMOTE_CONTROL_H
#define AIC_REMOTE_CONTROL_H
#include "motor_protocol.h"

/* User-approved bench policy. One step is 10 ms, PWM period is 8000. */
#define REMOTE_PWM_TARGET MOTOR_PWM_LIMIT
#define REMOTE_PWM_STEP 32U
#define REMOTE_RUN_TICKS 500U
#define REMOTE_LEASE_TICKS 100U
/* Stop reasons: boot=0, command=1, B21=2, 5s=3, heartbeat=4, pulse done=5. */
typedef enum { RES_OK, RES_DUPLICATE, RES_STALE, RES_BUSY, RES_INVALID } MotorResult;
typedef struct {
    bool running, has_sequence;
    bool button_candidate, button_stable;
    uint8_t button_ticks, stop_reason;
    uint16_t pwm, run_ticks, lease_ticks;
    uint32_t last_sequence;
    MotorCommand last_command;
    int16_t left_pwm, right_pwm, target_left, target_right;
    int16_t last_left, last_right;
    uint16_t duration_ticks, last_duration;
} RemoteControl;

static inline uint16_t remote_magnitude(int16_t value)
{
    return (uint16_t)(value < 0 ? -value : value);
}

static inline void remote_stop(RemoteControl *s, uint8_t reason)
{
    s->running = false;
    s->pwm = 0;
    s->left_pwm = s->right_pwm = s->target_left = s->target_right = 0;
    s->stop_reason = reason;
}
static inline MotorResult remote_request(RemoteControl *s, const MotorParser *request)
{
    uint32_t sequence = request->sequence;
    MotorCommand cmd = request->command;
    if (cmd == CMD_DRIVE && (remote_magnitude(request->left_pwm) > MOTOR_PWM_LIMIT ||
        remote_magnitude(request->right_pwm) > MOTOR_PWM_LIMIT ||
        !request->duration_ticks || request->duration_ticks > REMOTE_RUN_TICKS)) return RES_INVALID;
    if (cmd == CMD_STATUS) return RES_OK; /* Does not renew the run lease. */
    if (s->has_sequence && (int32_t)(sequence - s->last_sequence) <= 0) {
        /* Even a delayed STOP removes drive. Never restart or renew on replay. */
        if (cmd == CMD_STOP) remote_stop(s, 1);
        bool same = sequence == s->last_sequence && cmd == s->last_command;
        if (cmd == CMD_DRIVE) same = same && request->left_pwm == s->last_left &&
            request->right_pwm == s->last_right && request->duration_ticks == s->last_duration;
        return same ? RES_DUPLICATE : RES_STALE;
    }
    s->has_sequence = true;
    s->last_sequence = sequence;
    s->last_command = cmd;
    s->last_left = request->left_pwm; s->last_right = request->right_pwm;
    s->last_duration = request->duration_ticks;
    if (cmd == CMD_STOP) remote_stop(s, 1);
    else if (cmd == CMD_HEARTBEAT) {
        if (s->running) s->lease_ticks = 0;
    } else if (cmd == CMD_START || cmd == CMD_DRIVE) {
        if (cmd == CMD_DRIVE && !request->left_pwm && !request->right_pwm) {
            remote_stop(s, 1);
            return RES_OK;
        }
        if (s->running) return RES_BUSY;
        if (s->button_stable || s->button_candidate) return RES_BUSY;
        s->running = true;
        s->pwm = s->run_ticks = s->lease_ticks = 0;
        s->left_pwm = s->right_pwm = 0;
        s->target_left = cmd == CMD_START ? REMOTE_PWM_TARGET : request->left_pwm;
        s->target_right = cmd == CMD_START ? REMOTE_PWM_TARGET : request->right_pwm;
        s->duration_ticks = cmd == CMD_START ? REMOTE_RUN_TICKS : request->duration_ticks;
        s->stop_reason = 0;
    }
    return RES_OK;
}
static inline MotorResult remote_command(RemoteControl *s, uint32_t sequence, MotorCommand cmd)
{
    MotorParser request = {0};
    request.sequence = sequence; request.command = cmd;
    return remote_request(s, &request);
}
static inline int16_t remote_ramp(int16_t value, int16_t target)
{
    int delta = target - value;
    if (delta > (int)REMOTE_PWM_STEP) delta = REMOTE_PWM_STEP;
    if (delta < -(int)REMOTE_PWM_STEP) delta = -(int)REMOTE_PWM_STEP;
    return (int16_t)(value + delta);
}
static inline void remote_step(RemoteControl *s, bool button_pressed)
{
    if (button_pressed != s->button_candidate) {
        s->button_candidate = button_pressed;
        s->button_ticks = 1;
    } else if (s->button_ticks < 3) ++s->button_ticks;
    if (s->button_ticks == 3) s->button_stable = button_pressed;
    if (s->button_stable) {
        if (s->running) remote_stop(s, 2);
        return;
    }
    if (!s->running) return;
    ++s->run_ticks;
    ++s->lease_ticks;
    if (s->run_ticks >= REMOTE_RUN_TICKS) remote_stop(s, 3);
    else if (s->lease_ticks >= REMOTE_LEASE_TICKS) remote_stop(s, 4);
    else if (s->run_ticks >= s->duration_ticks) remote_stop(s, 5);
    else {
        s->left_pwm = remote_ramp(s->left_pwm, s->target_left);
        s->right_pwm = remote_ramp(s->right_pwm, s->target_right);
        uint16_t left = remote_magnitude(s->left_pwm), right = remote_magnitude(s->right_pwm);
        s->pwm = left > right ? left : right;
    }
}
#endif
