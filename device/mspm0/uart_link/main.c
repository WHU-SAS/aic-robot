/* Standalone MSPM0G3507 UART1 link test; no motor PWM or motion commands.
 * PA08 = UART1 TX, PA09 = UART1 RX, 115200 8N1.
 * UART0 PA10/PA11 provides optional USB debug logs at the same baud.
 * Uses the shared repository SDK in third_party/ti_mspm0_sdk/.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "uart_request_parser.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

volatile uint32_t g_received_lines;
volatile uint32_t g_acknowledged_lines;

static void put_string(UART_Regs *uart, const char *text)
{
    while (*text != '\0') {
        DL_UART_Main_transmitDataBlocking(uart, (uint8_t)*text++);
    }
}

static void init_uart(UART_Regs *uart)
{
    DL_UART_Main_ClockConfig clock_config = {
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
    DL_UART_Main_setClockConfig(uart, &clock_config);
    DL_UART_Main_init(uart, &config);
    /* BUSCLK = 32 MHz. 32 MHz/(16*(17+23/64)) = 115211.5 baud. */
    DL_UART_Main_setOversampling(uart, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(uart, 17U, 23U);
    DL_UART_Main_enableFIFOs(uart);
    DL_UART_Main_disableLoopbackMode(uart);
    DL_UART_Main_enable(uart);
}

static void init_board(void)
{
    DL_GPIO_reset(GPIOA);
    DL_UART_Main_reset(UART0);
    DL_UART_Main_reset(UART1);
    DL_GPIO_enablePower(GPIOA);
    DL_UART_Main_enablePower(UART0);
    DL_UART_Main_enablePower(UART1);
    delay_cycles(32U);

    /* Reset/default SYSOSC is 32 MHz; no crystal or PLL is needed. */
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);

    /* Hold the documented TB6612 STBY (PA12) LOW; no PWM is configured. */
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM34);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_12);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM19, IOMUX_PINCM19_PF_UART1_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM20, IOMUX_PINCM20_PF_UART1_RX);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);
    init_uart(UART0);
    init_uart(UART1);
}

int main(void)
{
    UartRequestParser parser = {0};
    uint8_t first_bytes[32];
    size_t first_used = 0U;
    bool first_reported = false;
    static const char hex[] = "0123456789ABCDEF";
    init_board();
    put_string(UART0, "MSPM0_UART1_TEST_READY 115200 8N1 TX=PA08 RX=PA09\r\n");
    for (;;) {
        if (DL_UART_Main_isRXFIFOEmpty(UART1)) continue;
        uint8_t value = DL_UART_Main_receiveData(UART1);
        if (!first_reported && first_used < sizeof(first_bytes)) {
            first_bytes[first_used++] = value;
        }
        if (!uart_request_feed(&parser, value)) continue;
        g_received_lines++;
        put_string(UART0, "UART1_RX ");
        put_string(UART0, parser.line);
        put_string(UART0, "\r\n");
        put_string(UART1, "MSPM0_ACK,");
        put_string(UART1, parser.line + UART_REQUEST_PREFIX_LEN);
        put_string(UART1, "\n");
        g_acknowledged_lines++;
        put_string(UART0, "UART1_ACK_SENT\r\n");
        if (!first_reported) {
            put_string(UART0, "UART1_FIRST_RX_HEX ");
            for (size_t i = 0; i < first_used; ++i) {
                char pair[] = {hex[first_bytes[i] >> 4], hex[first_bytes[i] & 15U], ' ', '\0'};
                put_string(UART0, pair);
            }
            put_string(UART0, "\r\n");
            first_reported = true;
        }
    }
}
