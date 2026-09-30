#include "demo_control.h"

void demo_control_init(DemoControl *s)
{
    *s = (DemoControl){0};
    /* A held button during reset cannot start the motors. */
    s->stable_pressed = true;
    s->candidate_pressed = true;
}

void demo_control_step(DemoControl *s, bool pressed)
{
    if (pressed != s->candidate_pressed) {
        s->candidate_pressed = pressed;
        s->debounce = 1;
    } else if (s->debounce < DEMO_DEBOUNCE_TICKS) {
        ++s->debounce;
    }
    if (s->debounce == DEMO_DEBOUNCE_TICKS &&
        s->stable_pressed != pressed) {
        s->stable_pressed = pressed;
        if (!pressed) {
            s->armed = true;
        } else if (s->armed) {
            s->armed = false;
            s->running = !s->running;
            s->run_ticks = 0;
            s->pwm = 0;
            s->stop_reason = s->running ? 0 : 1;
        }
    }
    if (s->running) {
        if (++s->run_ticks >= DEMO_RUN_TICKS) {
            s->running = false;
            s->pwm = 0;
            s->stop_reason = 2;
        } else if (s->pwm < DEMO_PWM_TARGET) {
            unsigned next = s->pwm + DEMO_PWM_STEP;
            s->pwm = next > DEMO_PWM_TARGET ? DEMO_PWM_TARGET : next;
        }
    } else {
        s->pwm = 0;
    }
}
