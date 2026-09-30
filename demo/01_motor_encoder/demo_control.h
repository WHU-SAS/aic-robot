#ifndef DEMO_CONTROL_H
#define DEMO_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

/* One step = 10 ms. PWM period is 8000 timer clocks (10 kHz). */
#define DEMO_PWM_TARGET 1600U /* 20% duty; NOT a guaranteed wheel speed */
#define DEMO_PWM_STEP 32U     /* 500 ms ramp */
#define DEMO_RUN_TICKS 500U   /* stop after 5 seconds */
#define DEMO_DEBOUNCE_TICKS 3U

typedef struct {
    bool running;
    bool armed;
    bool stable_pressed;
    bool candidate_pressed;
    uint8_t debounce;
    uint16_t run_ticks;
    uint16_t pwm;
    uint8_t stop_reason; /* 0: boot, 1: button, 2: timeout */
} DemoControl;

void demo_control_init(DemoControl *s);
void demo_control_step(DemoControl *s, bool pressed);
#endif
