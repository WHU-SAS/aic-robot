/* K230 UART1 remote bench control. Motor wiring/config matches motor_encoder.
 * Startup has no drive. No mechanical-arm commands are sent by this firmware. */
#include "../motor_encoder/ti_msp_dl_config.h"
#include "remote_control.h"
#include <stdio.h>

#define MOTOR_A_REVERSE 0
#define MOTOR_B_REVERSE 0
static RemoteControl control;
static volatile int32_t pending_a, pending_b;
static volatile uint32_t elapsed_ms, encoder_a_total, encoder_b_total;

static void motor_output(bool enabled, int16_t left, int16_t right)
{
    if (!enabled || (!left && !right)) {
        DL_GPIO_clearPins(STBY_PORT, STBY_stby_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
        return;
    }
    bool reverse_a = (left < 0) != (MOTOR_A_REVERSE != 0);
    bool reverse_b = (right < 0) != (MOTOR_B_REVERSE != 0);
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    if (left) DL_GPIO_setPins(AIN_PORT, reverse_a ? AIN_AIN2_PIN : AIN_AIN1_PIN);
    if (right) DL_GPIO_setPins(BIN_PORT, reverse_b ? BIN_BIN2_PIN : BIN_BIN1_PIN);
    uint16_t a = remote_magnitude(left), b = remote_magnitude(right);
    if (a > REMOTE_PWM_TARGET) a = REMOTE_PWM_TARGET;
    if (b > REMOTE_PWM_TARGET) b = REMOTE_PWM_TARGET;
    DL_Timer_setCaptureCompareValue(PWM_0_INST, a, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, b, GPIO_PWM_0_C1_IDX);
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
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) != DL_TIMER_IIDX_ZERO) return;
    /* Same priority as encoder IRQ: atomic read/reset, no serial in ISRs. */
    encoder_a_total += (uint32_t)pending_a;
    encoder_b_total += (uint32_t)pending_b;
    pending_a = pending_b = 0;
    elapsed_ms += 10;
    remote_step(&control, DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) == 0);
    motor_output(control.running, control.left_pwm, control.right_pwm);
}

static void init_uart1(void)
{
    DL_UART_Main_reset(UART1);
    DL_UART_Main_enablePower(UART1);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM19, IOMUX_PINCM19_PF_UART1_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM20, IOMUX_PINCM20_PF_UART1_RX);
    DL_UART_Main_ClockConfig clock = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    DL_UART_Main_Config config = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE
    };
    DL_UART_Main_setClockConfig(UART1, &clock);
    DL_UART_Main_init(UART1, &config);
    /* Motor firmware uses 80 MHz MCLK / 40 MHz UART BUSCLK, not 32 MHz. */
    DL_UART_Main_setOversampling(UART1, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART1, 21, 45);
    DL_UART_Main_enableFIFOs(UART1);
    DL_UART_Main_disableLoopbackMode(UART1);
    DL_UART_Main_enable(UART1);
}
static void put_string(UART_Regs *uart, const char *text)
{
    while (*text) DL_UART_Main_transmitDataBlocking(uart, (uint8_t)*text++);
}
static void respond(const MotorParser *request)
{
    char body[160], frame[180];
    static const char *const names[] = {"STATUS", "START", "STOP", "HB", "DRIVE"};
    RemoteControl snapshot;
    uint32_t ms, a, b;
    __disable_irq();
    MotorResult result = remote_request(&control, request);
    motor_output(control.running, control.left_pwm, control.right_pwm); /* STOP before ACK. */
    snapshot = control;
    ms = elapsed_ms; a = encoder_a_total; b = encoder_b_total;
    __enable_irq();
    snprintf(body, sizeof(body), "R,%08lX,%s,%u,%lu,%u,%u,%ld,%ld,%u,%08lX,2,%d,%d,%d,%d,%u",
        (unsigned long)request->sequence, names[request->command], (unsigned)result,
        (unsigned long)ms, (unsigned)snapshot.running, (unsigned)snapshot.pwm,
        (long)(int32_t)a, (long)(int32_t)b, (unsigned)snapshot.stop_reason,
        (unsigned long)snapshot.last_sequence,
        (int)snapshot.left_pwm, (int)snapshot.right_pwm,
        (int)snapshot.target_left, (int)snapshot.target_right,
        snapshot.running ? (unsigned)(snapshot.duration_ticks - snapshot.run_ticks) * 10U : 0U);
    snprintf(frame, sizeof(frame), "@%s,%04X\n", body, (unsigned)motor_crc(body, strlen(body)));
    put_string(UART1, frame);
}
int main(void)
{
    MotorParser parser = {0};
    SYSCFG_DL_init();
    init_uart1();
    control.button_candidate = control.button_stable =
        DL_GPIO_readPins(KEY_PORT, KEY_key_PIN) == 0;
    motor_output(false, 0, 0);
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
    put_string(UART0, "AIC_MOTOR_UART_READY v2 115200 PWM=0 STBY=0\r\n");
    for (;;) {
        if (!DL_UART_Main_isRXFIFOEmpty(UART1) &&
            motor_feed(&parser, DL_UART_Main_receiveData(UART1))) respond(&parser);
    }
}
