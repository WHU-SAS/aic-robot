#ifndef AIC_REMOTE_CONTROL_H
#define AIC_REMOTE_CONTROL_H
#include "motor_protocol.h"

/* User-approved bench policy. One step is 10 ms, PWM period is 8000. */
#define REMOTE_PWM_TARGET MOTOR_PWM_LIMIT
#define REMOTE_PWM_STEP 32U
#define REMOTE_RUN_TICKS 500U
#define REMOTE_FOLLOW_RUN_TICKS 2000U
#define REMOTE_LEASE_TICKS 100U
/* Stop reasons: boot=0, command=1, B21=2, deadline=3, heartbeat=4, pulse=5, distance=6, angle=7, arc=8.
 * Deadline is 5s for existing modes, 20s only for FOLLOW. */
typedef enum { RES_OK, RES_DUPLICATE, RES_STALE, RES_BUSY, RES_INVALID } MotorResult;
typedef struct {
    bool running, has_sequence, follow_mode, arc_mode;
    bool button_candidate, button_stable;
    uint8_t button_ticks, stop_reason;
    uint16_t pwm, run_ticks, lease_ticks;
    uint32_t last_sequence;
    MotorCommand last_command;
    int16_t left_pwm, right_pwm, target_left, target_right;
    int16_t last_left, last_right;
    uint16_t duration_ticks, last_duration;
    uint16_t distance_mm, last_distance_mm;
    bool reverse_move, last_reverse_move;
    int16_t angle_deg, last_angle_deg;
    uint16_t left_speed, right_speed, last_left_speed, last_right_speed;
    int16_t arc_left_mm, arc_right_mm, last_arc_left_mm, last_arc_right_mm;
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
    s->distance_mm = 0;
    s->reverse_move = false;
    s->angle_deg = 0;
    s->follow_mode = false;
    s->arc_mode = false;
    s->arc_left_mm = s->arc_right_mm = 0;
    s->left_speed = s->right_speed = 0;
    s->stop_reason = reason;
}
static inline MotorResult remote_request(RemoteControl *s, const MotorParser *request)
{
    uint32_t sequence = request->sequence;
    MotorCommand cmd = request->command;
    if (cmd == CMD_DRIVE && (remote_magnitude(request->left_pwm) > MOTOR_PWM_LIMIT ||
        remote_magnitude(request->right_pwm) > MOTOR_PWM_LIMIT ||
        !request->duration_ticks || request->duration_ticks > REMOTE_RUN_TICKS)) return RES_INVALID;
    if ((cmd == CMD_FOLLOW || cmd == CMD_SPEED) &&
        (request->left_speed > 80 || request->right_speed > 80)) return RES_INVALID;
    if (cmd == CMD_STATUS) return RES_OK; /* Does not renew the run lease. */
    if (s->has_sequence && (int32_t)(sequence - s->last_sequence) <= 0) {
        /* Even a delayed STOP removes drive. Never restart or renew on replay. */
        if (cmd == CMD_STOP) remote_stop(s, 1);
        bool same = sequence == s->last_sequence && cmd == s->last_command;
        if (cmd == CMD_DRIVE) same = same && request->left_pwm == s->last_left &&
            request->right_pwm == s->last_right && request->duration_ticks == s->last_duration;
        if (cmd == CMD_MOVE) same = same && request->distance_mm == s->last_distance_mm &&
            request->reverse_move == s->last_reverse_move;
        if (cmd == CMD_TURN) same = same && request->angle_deg == s->last_angle_deg;
        if (cmd == CMD_ARC) same = same && request->arc_left_mm == s->last_arc_left_mm &&
            request->arc_right_mm == s->last_arc_right_mm;
        if (cmd == CMD_FOLLOW || cmd == CMD_SPEED) same = same &&
            request->left_speed == s->last_left_speed && request->right_speed == s->last_right_speed;
        return same ? RES_DUPLICATE : RES_STALE;
    }
    s->has_sequence = true;
    s->last_sequence = sequence;
    s->last_command = cmd;
    s->last_left = request->left_pwm; s->last_right = request->right_pwm;
    s->last_duration = request->duration_ticks;
    s->last_distance_mm = request->distance_mm;
    s->last_reverse_move = request->reverse_move;
    s->last_angle_deg = request->angle_deg;
    s->last_left_speed = request->left_speed; s->last_right_speed = request->right_speed;
    s->last_arc_left_mm = request->arc_left_mm; s->last_arc_right_mm = request->arc_right_mm;
    if (cmd == CMD_STOP) remote_stop(s, 1);
    else if (cmd == CMD_HEARTBEAT) {
        if (s->running) s->lease_ticks = 0;
    } else if (cmd == CMD_SPEED) {
        /* An update never starts or restarts a stopped task. */
        if (!s->running || !s->follow_mode) return RES_BUSY;
        s->left_speed = request->left_speed; s->right_speed = request->right_speed;
        s->lease_ticks = 0;
    } else if (cmd == CMD_START || cmd == CMD_DRIVE || cmd == CMD_MOVE || cmd == CMD_TURN || cmd == CMD_FOLLOW || cmd == CMD_ARC) {
        if (cmd == CMD_DRIVE && !request->left_pwm && !request->right_pwm) {
            remote_stop(s, 1);
            return RES_OK;
        }
        if (s->running) return RES_BUSY;
        if (s->button_stable || s->button_candidate) return RES_BUSY;
        if (cmd == CMD_ARC && !request->arc_left_mm && !request->arc_right_mm) {
            remote_stop(s, 8); /* Explicit zero-distance capability query. */
            return RES_OK;
        }
        s->running = true;
        s->pwm = s->run_ticks = s->lease_ticks = 0;
        s->left_pwm = s->right_pwm = 0;
        s->target_left = cmd == CMD_START ? REMOTE_PWM_TARGET : request->left_pwm;
        s->target_right = cmd == CMD_START ? REMOTE_PWM_TARGET : request->right_pwm;
        s->follow_mode = cmd == CMD_FOLLOW;
        s->arc_mode = cmd == CMD_ARC;
        s->arc_left_mm = s->arc_mode ? request->arc_left_mm : 0;
        s->arc_right_mm = s->arc_mode ? request->arc_right_mm : 0;
        s->duration_ticks = cmd == CMD_DRIVE ? request->duration_ticks :
            (s->follow_mode ? REMOTE_FOLLOW_RUN_TICKS : REMOTE_RUN_TICKS);
        s->distance_mm = cmd == CMD_MOVE ? request->distance_mm : 0;
        s->reverse_move = cmd == CMD_MOVE && request->reverse_move;
        s->angle_deg = cmd == CMD_TURN ? request->angle_deg : 0;
        s->left_speed = s->follow_mode ? request->left_speed : 0;
        s->right_speed = s->follow_mode ? request->right_speed : 0;
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
    if (s->run_ticks >= (s->follow_mode ? REMOTE_FOLLOW_RUN_TICKS : REMOTE_RUN_TICKS)) remote_stop(s, 3);
    else if (s->lease_ticks >= REMOTE_LEASE_TICKS) remote_stop(s, 4);
    else if (s->run_ticks >= s->duration_ticks) remote_stop(s, 5);
    else if (!s->distance_mm && !s->angle_deg && !s->follow_mode && !s->arc_mode) {
        s->left_pwm = remote_ramp(s->left_pwm, s->target_left);
        s->right_pwm = remote_ramp(s->right_pwm, s->target_right);
        uint16_t left = remote_magnitude(s->left_pwm), right = remote_magnitude(s->right_pwm);
        s->pwm = left > right ? left : right;
    }
}
#endif
