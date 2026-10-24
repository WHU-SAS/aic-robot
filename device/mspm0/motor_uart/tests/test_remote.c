#include "remote_control.h"
#include <assert.h>
#include <stdio.h>

static unsigned feed(MotorParser *p, const char *text)
{
    unsigned frames = 0;
    while (*text) frames += motor_feed(p, (uint8_t)*text++);
    return frames;
}
static void frame(char *out, uint32_t seq, const char *cmd)
{
    char body[48];
    snprintf(body, sizeof(body), "M,%08X,%s", (unsigned)seq, cmd);
    snprintf(out, 64, "@%s,%04X\n", body, motor_crc(body, strlen(body)));
}
int main(void)
{
    MotorParser p = {0};
    char start[64], hb[64], stop[64], status[64];
    frame(start, 1, "START"); frame(hb, 2, "HB");
    frame(stop, 3, "STOP"); frame(status, 0, "STATUS");
    assert(motor_crc("123456789", 9) == 0x29B1);
    assert(feed(&p, "noise") == 0);
    /* Include actual cold-start NUL bytes rather than relying on strlen. */
    assert(!motor_feed(&p, 0)); assert(feed(&p, start) == 1);
    assert(p.sequence == 1 && p.command == CMD_START);
    assert(feed(&p, hb) == 1 && p.command == CMD_HEARTBEAT);
    assert(feed(&p, "@M,00000003,START,0000\n") == 0);
    assert(feed(&p, "@M,00000003,BAD,0000\n") == 0);
    assert(feed(&p, "@M,00000001,STA") == 0);
    assert(feed(&p, stop) == 1 && p.command == CMD_STOP);
    assert(feed(&p, "@zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz") == 0);
    assert(feed(&p, status) == 1 && p.command == CMD_STATUS);
    for (size_t n = 0; n < strlen(start); ++n) {
        MotorParser partial = {0};
        for (size_t i = 0; i < n; ++i) assert(!motor_feed(&partial, start[i]));
        assert(feed(&partial, stop) == 1 && partial.command == CMD_STOP);
    }
    RemoteControl s = {0};
    assert(!s.running && s.pwm == 0);
    assert(remote_command(&s, 1, CMD_START) == RES_OK);
    for (unsigned n = 0; n < 50; ++n) remote_step(&s, false);
    assert(s.running && s.pwm == 1600 && s.run_ticks == 50);
    assert(remote_command(&s, 1, CMD_START) == RES_DUPLICATE);
    assert(s.run_ticks == 50);
    assert(remote_command(&s, 2, CMD_START) == RES_BUSY && s.run_ticks == 50);
    assert(remote_command(&s, 1, CMD_START) == RES_STALE);
    for (unsigned n = 50; n < 500; ++n) {
        if (n % 20 == 0) assert(remote_command(&s, 100+n, CMD_HEARTBEAT) == RES_OK);
        remote_step(&s, false);
    }
    assert(!s.running && s.pwm == 0 && s.stop_reason == 3);
    assert(remote_command(&s, 1000, CMD_START) == RES_OK);
    for (unsigned n = 0; n < 99; ++n) {
        remote_command(&s, 9999, CMD_STATUS);
        remote_step(&s, false);
    }
    assert(s.running);
    remote_step(&s, false);
    assert(!s.running && s.stop_reason == 4);
    assert(remote_command(&s, 1001, CMD_HEARTBEAT) == RES_OK && !s.running);
    assert(remote_command(&s, 1002, CMD_START) == RES_OK);
    for (unsigned n = 0; n < 3; ++n) remote_step(&s, true);
    assert(!s.running && s.pwm == 0 && s.stop_reason == 2);
    assert(remote_command(&s, 1003, CMD_START) == RES_BUSY);
    for (unsigned n = 0; n < 3; ++n) remote_step(&s, false);
    assert(!s.running); /* button release never starts */
    assert(remote_command(&s, 1004, CMD_START) == RES_OK);
    assert(remote_command(&s, 2, CMD_STOP) == RES_STALE && !s.running);
    assert(remote_command(&s, 1004, CMD_START) == RES_DUPLICATE && !s.running);
    assert(remote_command(&s, 1005, CMD_START) == RES_OK);
    remote_step(&s, false);
    assert(remote_command(&s, 1006, CMD_STOP) == RES_OK && s.pwm == 0);
    /* A replayed heartbeat cannot keep a crashed sender's motor alive. */
    assert(remote_command(&s, 1007, CMD_START) == RES_OK);
    remote_step(&s, false);
    assert(remote_command(&s, 1008, CMD_HEARTBEAT) == RES_OK);
    for (unsigned n = 0; n < 100; ++n) {
        assert(remote_command(&s, 1008, CMD_HEARTBEAT) == RES_DUPLICATE);
        remote_step(&s, false);
    }
    assert(!s.running && s.stop_reason == 4);
    RemoteControl wrap = {0};
    assert(remote_command(&wrap, 0xFFFFFFFE, CMD_STOP) == RES_OK);
    assert(remote_command(&wrap, 0xFFFFFFFF, CMD_START) == RES_OK);
    assert(remote_command(&wrap, 0, CMD_STOP) == RES_OK && !wrap.running);
    /* Independent, signed wheels; exact payload replay; bounded pulses. */
    RemoteControl drive = {0};
    char motion[64];
    frame(motion, 1, "DRIVE,-800,400,300");
    assert(feed(&p, motion) == 1 && p.command == CMD_DRIVE);
    assert(p.left_pwm == -800 && p.right_pwm == 400 && p.duration_ticks == 30);
    assert(remote_request(&drive, &p) == RES_OK);
    for (unsigned n = 0; n < 20; ++n) remote_step(&drive, false);
    assert(drive.left_pwm == -640 && drive.right_pwm == 400 && drive.pwm == 640);
    assert(remote_request(&drive, &p) == RES_DUPLICATE && drive.run_ticks == 20);
    p.right_pwm = 401;
    assert(remote_request(&drive, &p) == RES_STALE && drive.target_right == 400);
    p.right_pwm = 400; p.sequence = 2;
    assert(remote_request(&drive, &p) == RES_BUSY && drive.run_ticks == 20);
    for (unsigned n = 20; n < 30; ++n) remote_step(&drive, false);
    assert(!drive.running && drive.stop_reason == 5 && !drive.left_pwm && !drive.right_pwm);
    assert(remote_command(&drive, 3, CMD_HEARTBEAT) == RES_OK && !drive.running);
    frame(motion, 4, "DRIVE,0,-1600,2000");
    assert(feed(&p, motion) == 1 && remote_request(&drive, &p) == RES_OK);
    for (unsigned n = 0; n < 100; ++n) remote_step(&drive, false);
    assert(!drive.running && drive.stop_reason == 4 && !drive.right_pwm);
    frame(motion, 5, "DRIVE,500,0,1000");
    assert(feed(&p, motion) == 1 && remote_request(&drive, &p) == RES_OK);
    for (unsigned n = 0; n < 10; ++n) remote_step(&drive, false);
    assert(drive.left_pwm == 320 && !drive.right_pwm);
    frame(motion, 6, "DRIVE,0,0,10");
    assert(feed(&p, motion) == 1 && remote_request(&drive, &p) == RES_OK);
    assert(!drive.running && !drive.left_pwm && !drive.right_pwm && drive.stop_reason == 1);
    const char *bad[] = {"DRIVE,1601,0,100", "DRIVE,-1601,0,100", "DRIVE,1,1,0",
        "DRIVE,1,1,5001", "DRIVE,1,1,11", "DRIVE,,1,100", "DRIVE,1,1,100,1",
        "DRIVE,+1,1,100", "DRIVE,1,1,-100", "DRIVE,1,1,100x"};
    for (unsigned n = 0; n < sizeof(bad)/sizeof(bad[0]); ++n) {
        frame(motion, 7, bad[n]);
        assert(feed(&p, motion) == 0);
    }
    frame(motion, 7, "DRIVE,-1600,1600,5000");
    assert(feed(&p, motion) == 1 && remote_request(&drive, &p) == RES_OK);
    for (unsigned n = 0; n < 500; ++n) {
        if (n % 20 == 0) remote_command(&drive, 100+n, CMD_HEARTBEAT);
        remote_step(&drive, false);
    }
    assert(!drive.running && drive.stop_reason == 3);
    frame(motion, 1000, "DRIVE,-500,-500,1000");
    assert(feed(&p, motion) == 1 && remote_request(&drive, &p) == RES_OK);
    for (unsigned n = 0; n < 3; ++n) remote_step(&drive, true);
    assert(!drive.running && drive.stop_reason == 2 && !drive.left_pwm && !drive.right_pwm);
    assert(remote_request(&drive, &p) == RES_DUPLICATE && !drive.running);
    puts("PASS: noise/CRC/framing, replay, ramp, stop, watchdog, deadline, B21, wrap");
}
