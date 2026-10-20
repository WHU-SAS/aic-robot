#ifndef AIC_REMOTE_CONTROL_H
#define AIC_REMOTE_CONTROL_H
#include "motor_protocol.h"

/* User-approved bench policy. One step is 10 ms, PWM period is 8000. */
#define REMOTE_PWM_TARGET 1600U
#define REMOTE_PWM_STEP 32U
#define REMOTE_RUN_TICKS 500U
#define REMOTE_LEASE_TICKS 100U
/* Stop reasons: boot=0, command=1, B21=2, 5s limit=3, heartbeat loss=4. */
typedef enum { RES_OK, RES_DUPLICATE, RES_STALE, RES_BUSY } MotorResult;
typedef struct {
    bool running, has_sequence;
    bool button_candidate, button_stable;
    uint8_t button_ticks, stop_reason;
    uint16_t pwm, run_ticks, lease_ticks;
    uint32_t last_sequence;
    MotorCommand last_command;
} RemoteControl;

static inline void remote_stop(RemoteControl *s, uint8_t reason)
{
    s->running = false;
    s->pwm = 0;
    s->stop_reason = reason;
}
static inline MotorResult remote_command(RemoteControl *s, uint32_t sequence, MotorCommand cmd)
{
    if (cmd == CMD_STATUS) return RES_OK; /* Does not renew the run lease. */
    if (s->has_sequence && (int32_t)(sequence - s->last_sequence) <= 0) {
        /* Even a delayed STOP removes drive. Never restart or renew on replay. */
        if (cmd == CMD_STOP) remote_stop(s, 1);
        return sequence == s->last_sequence && cmd == s->last_command ? RES_DUPLICATE : RES_STALE;
    }
    s->has_sequence = true;
    s->last_sequence = sequence;
    s->last_command = cmd;
    if (cmd == CMD_STOP) remote_stop(s, 1);
    else if (cmd == CMD_HEARTBEAT) {
        if (s->running) s->lease_ticks = 0;
    } else if (cmd == CMD_START) {
        if (s->running) return RES_BUSY;
        if (s->button_stable || s->button_candidate) return RES_BUSY;
        s->running = true;
        s->pwm = s->run_ticks = s->lease_ticks = 0;
        s->stop_reason = 0;
    }
    return RES_OK;
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
    else if (s->pwm < REMOTE_PWM_TARGET) {
        unsigned next = s->pwm + REMOTE_PWM_STEP;
        s->pwm = next > REMOTE_PWM_TARGET ? REMOTE_PWM_TARGET : next;
    }
}
#endif
