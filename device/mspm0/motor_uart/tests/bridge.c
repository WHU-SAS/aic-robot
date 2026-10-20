/* Offline UART simulator: production parser/control, synthetic encoder counts. */
#include "remote_control.h"
#include <stdio.h>
static RemoteControl state;
static MotorParser parser;
static uint32_t milliseconds;
static int32_t a, b;
static char response[180];
static bool button;
void bridge_button(int pressed) { button = pressed != 0; }
void bridge_reset(void)
{
    state = (RemoteControl){0}; parser = (MotorParser){0};
    milliseconds = 0; a = b = 0; button = false;
}
void bridge_tick(void)
{
    remote_step(&state, button);
    milliseconds += 10;
    if (state.pwm) { ++a; --b; }
}
const char *bridge_feed(uint8_t byte)
{
    if (!motor_feed(&parser, byte)) return "";
    static const char *const names[] = {"STATUS", "START", "STOP", "HB"};
    MotorResult result = remote_command(&state, parser.sequence, parser.command);
    char body[150];
    snprintf(body, sizeof(body), "R,%08X,%s,%u,%u,%u,%u,%d,%d,%u,%08X",
        (unsigned)parser.sequence, names[parser.command], (unsigned)result,
        (unsigned)milliseconds, state.running, state.pwm, (int)a, (int)b,
        state.stop_reason, (unsigned)state.last_sequence);
    snprintf(response, sizeof(response), "@%s,%04X\n", body, motor_crc(body, strlen(body)));
    return response;
}
