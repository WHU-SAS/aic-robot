#include "demo_control.h"
#include <assert.h>
#include <stdio.h>
static void ticks(DemoControl *s, bool pressed, unsigned n)
{
    while (n--) {
        demo_control_step(s, pressed);
        assert(s->pwm <= DEMO_PWM_TARGET);
        if (!s->running) assert(s->pwm == 0);
    }
}
int main(void)
{
    DemoControl s;
    demo_control_init(&s);
    ticks(&s, true, 1000); assert(!s.running); /* held at boot */
    ticks(&s, false, 3);
    ticks(&s, true, 1); ticks(&s, false, 1);
    ticks(&s, true, 2); ticks(&s, false, 3);
    assert(!s.running); /* contact bounce */
    ticks(&s, true, 3); assert(s.running && s.pwm == DEMO_PWM_STEP);
    ticks(&s, true, 49); assert(s.pwm == DEMO_PWM_TARGET);
    ticks(&s, false, 3); ticks(&s, true, 3);
    assert(!s.running && s.stop_reason == 1); /* stop press */
    ticks(&s, true, 1000); assert(!s.running); /* held cannot restart */
    ticks(&s, false, 3); ticks(&s, true, 3);
    assert(s.running && s.pwm == DEMO_PWM_STEP); /* ramp restarts */
    ticks(&s, true, DEMO_RUN_TICKS - 2); assert(s.running);
    ticks(&s, true, 1); assert(!s.running && s.stop_reason == 2);
    ticks(&s, true, 1000); assert(!s.running); /* timeout hold */
    ticks(&s, false, 3); ticks(&s, true, 3); assert(s.running);
    puts("PASS: boot hold, bounce, ramp, button stop, hold, timeout, restart");
}
