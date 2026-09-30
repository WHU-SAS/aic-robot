/* R3X bench demo, adapted from WHEELTEC C07A TB6612 example.
 * Board: LCKFB Tianmengxing MSPM0G3507; see README.md before wiring.
 * Only this file, demo_control.c and ti_msp_dl_config.c are application
 * build inputs. Original Hardware/ files are retained as references.
 */
#include "ti_msp_dl_config.h"
#include "demo_control.h"
#include <stdio.h>

#define MOTOR_A_REVERSE 0
#define MOTOR_B_REVERSE 0 /* corrected after the lifted-wheel direction test */

static DemoControl control;
static volatile int32_t pending_a, pending_b;
/* Watch these variables in the debugger. Totals wrap modulo 2^32. */
volatile uint32_t demo_ms, encoder_a_total, encoder_b_total;
volatile int32_t encoder_a_100ms, encoder_b_100ms;
volatile uint16_t demo_pwm;
volatile uint8_t demo_running, demo_stop_reason;
static volatile uint32_t report_sequence;

static void motor_output(bool enabled, uint16_t pwm)
{
    if (!enabled || pwm == 0) {
        DL_GPIO_clearPins(STBY_PORT, STBY_stby_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
        return;
    }
    if (pwm > DEMO_PWM_TARGET) pwm = DEMO_PWM_TARGET;
    DL_GPIO_clearPins(AIN_PORT, MOTOR_A_REVERSE ? AIN_AIN1_PIN : AIN_AIN2_PIN);
    DL_GPIO_setPins(AIN_PORT, MOTOR_A_REVERSE ? AIN_AIN2_PIN : AIN_AIN1_PIN);
    DL_GPIO_clearPins(BIN_PORT, MOTOR_B_REVERSE ? BIN_BIN1_PIN : BIN_BIN2_PIN);
    DL_GPIO_setPins(BIN_PORT, MOTOR_B_REVERSE ? BIN_BIN2_PIN : BIN_BIN1_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, pwm, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, pwm, GPIO_PWM_0_C1_IDX);
    DL_GPIO_setPins(STBY_PORT, STBY_stby_PIN);
}

/* Rising edges of BOTH phases: 2x decoding, matching the configuration.
 * Both bits pending together means timing is ambiguous: skip that sample.
 * This is a low-speed teaching demo, not a lossless high-speed decoder.
 */
static int edge_delta(uint32_t flags, uint32_t pins, uint32_t a, uint32_t b)
{
    if (flags == a) return (pins & b) ? 1 : -1;
    if (flags == b) return (pins & a) ? -1 : 1;
    return 0;
}

void GROUP1_IRQHandler(void)
{
    const uint32_t a_mask = ENCODERA_E1A_PIN | ENCODERA_E1B_PIN;
    const uint32_t b_mask = ENCODERB_E2A_PIN | ENCODERB_E2B_PIN;
    uint32_t a = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT, a_mask);
    uint32_t b = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT, b_mask);
    uint32_t a_pins = DL_GPIO_readPins(ENCODERA_PORT, a_mask);
    uint32_t b_pins = DL_GPIO_readPins(ENCODERB_PORT, b_mask);
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, a);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, b);
    pending_a += edge_delta(a, a_pins, ENCODERA_E1A_PIN, ENCODERA_E1B_PIN);
    pending_b += edge_delta(b, b_pins, ENCODERB_E2A_PIN, ENCODERB_E2B_PIN);
}

void TIMER_0_INST_IRQHandler(void)
{
    static int32_t window_a, window_b;
    static unsigned window_ticks;
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) != DL_TIMER_IIDX_ZERO) return;
    /* Encoder and timer share priority 0, so this read/reset is atomic
       against the encoder ISR. No serial printing inside interrupts. */
    int32_t a = pending_a, b = pending_b;
    pending_a = pending_b = 0;
    encoder_a_total += (uint32_t)a;
    encoder_b_total += (uint32_t)b;
    window_a += a;
    window_b += b;
    demo_ms += 10;
    demo_control_step(&control, DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) == 0);
    motor_output(control.running, control.pwm);
    demo_pwm = control.pwm;
    demo_running = control.running;
    demo_stop_reason = control.stop_reason;
    if (++window_ticks == 10) {
        window_ticks = 0;
        encoder_a_100ms = window_a;
        encoder_b_100ms = window_b;
        window_a = window_b = 0;
        ++report_sequence;
    }
}

static void uart_puts(const char *s)
{
    while (*s) DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)*s++);
}

int main(void)
{
    uint32_t last_report = 0;
    SYSCFG_DL_init();
    demo_control_init(&control);
    motor_output(false, 0);
    /* Explicit high-at-zero, low-at-compare PWM: pwm / 8000 duty. */
    DL_Timer_setCaptureCompareAction(PWM_0_INST,
        DL_TIMER_CC_ZACT_CCP_HIGH | DL_TIMER_CC_CUACT_CCP_LOW, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareAction(PWM_0_INST,
        DL_TIMER_CC_ZACT_CCP_HIGH | DL_TIMER_CC_CUACT_CCP_LOW, GPIO_PWM_0_C1_IDX);
    DL_Timer_startCounter(PWM_0_INST);
    NVIC_SetPriority(ENCODERA_INT_IRQN, 0);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 0);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_Timer_startCounter(TIMER_0_INST);
    __enable_irq();
    uart_puts("R3X bench demo: B21=start/stop, duty=20%, timeout=5s\r\n");
    uart_puts("ms,run,pwm,a_100ms,b_100ms,a_total,b_total,stop_reason\r\n");
    for (;;) {
        if (report_sequence != last_report) {
            char line[160];
            uint32_t ms, total_a, total_b;
            int32_t a, b;
            unsigned run, pwm, reason;
            __disable_irq();
            last_report = report_sequence;
            ms = demo_ms; total_a = encoder_a_total; total_b = encoder_b_total;
            a = encoder_a_100ms; b = encoder_b_100ms;
            run = demo_running; pwm = demo_pwm; reason = demo_stop_reason;
            __enable_irq();
            snprintf(line, sizeof(line), "%lu,%u,%u,%ld,%ld,%ld,%ld,%u\r\n",
                (unsigned long)ms, run, pwm, (long)a, (long)b,
                (long)(int32_t)total_a, (long)(int32_t)total_b, reason);
            uart_puts(line);
        }
    }
}
