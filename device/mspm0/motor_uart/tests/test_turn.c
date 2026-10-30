#include "turn_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static bool parse_turn(MotorParser *p, uint32_t sequence, const char *angle)
{
    char body[48], frame[64];
    snprintf(body, sizeof(body), "M,%08X,TURN,%s", (unsigned)sequence, angle);
    snprintf(frame, sizeof(frame), "@%s,%04X\n", body, motor_crc(body, strlen(body)));
    unsigned complete = 0;
    for (const char *q = frame; *q; ++q) complete += motor_feed(p, (uint8_t)*q);
    return complete == 1;
}

static double simulate(int degrees, double rolling, uint32_t origin, bool stiction, double coast_s)
{
    RemoteControl motor = {0}; TurnControl turn;
    MotorParser request = {.sequence=1, .command=CMD_TURN, .angle_deg=degrees};
    assert(remote_request(&motor, &request) == RES_OK);
    turn_begin(&turn, origin, origin);
    double left=0, right=0, vl=0, vr=0;
    unsigned tick=0, first_motion_tick=500;
    bool moving_left=!stiction, moving_right=!stiction;
    for (; tick < 500 && motor.running; ++tick) {
        if (tick % 10 == 0) remote_command(&motor, 2+tick, CMD_HEARTBEAT);
        int16_t previous_left=motor.left_pwm, previous_right=motor.right_pwm;
        remote_step(&motor, false);
        turn_step(&turn, &motor,
            origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT),
            origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT));
        assert(motor.left_pwm >= 0 && motor.left_pwm <= 1600);
        assert(motor.right_pwm >= 0 && motor.right_pwm <= 1600);
        assert(degrees > 0 ? !motor.left_pwm : !motor.right_pwm);
        if (motor.running) {
            assert(abs(motor.left_pwm-previous_left) <= 32);
            assert(abs(motor.right_pwm-previous_right) <= 32);
        }
        if (motor.left_pwm >= 1400) moving_left=true;
        if (motor.right_pwm >= 1400) moving_right=true;
        double target_l=moving_left ? fmax(0, motor.left_pwm-900)*0.20 : 0;
        double target_r=moving_right ? fmax(0, motor.right_pwm-1000)*0.18 : 0;
        if (degrees > 0) target_l=target_r*rolling;
        else target_r=target_l*rolling;
        vl += (target_l-vl)*0.01/0.08;
        vr += (target_r-vr)*0.01/0.08;
        if ((vl > 0 || vr > 0) && first_motion_tick==500) first_motion_tick=tick;
        left+=vl*0.01; right+=vr*0.01;
    }
    assert(!motor.running && motor.stop_reason == 7 && tick < 500);
    if (stiction) assert(first_motion_tick < 100); /* Start within1s in this model. */
    for (unsigned n=0; n<200; ++n) {
        vl += -vl*0.01/coast_s; vr += -vr*0.01/coast_s;
        left+=vl*0.01; right+=vr*0.01;
        remote_step(&motor, false); turn_step(&turn, &motor, origin, origin);
        assert(!motor.running && !motor.left_pwm && !motor.right_pwm && !motor.angle_deg);
    }
    double angle=(right-left)/TURN_MM_PER_DEG;
    printf("turn model target=%d rolling=%.2f stiction=%d coast=%.2fs: %.3fdeg, %ums\n", degrees,rolling,stiction,coast_s,angle,tick*10);
    /* A model sanity bound, not an acceptance tolerance for the real car. */
    assert(angle*degrees > 0 && fabs(angle-degrees) < 2);
    return fabs(angle);
}

int main(void)
{
    MotorParser p={0}; RemoteControl motor={0}; TurnControl turn;
    const char *bad[]={"0", "-0", "31", "-31", "+15", "1.5", "15,1", ""};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) assert(!parse_turn(&p,1,bad[i]));
    assert(parse_turn(&p,1,"15") && p.command==CMD_TURN && p.angle_deg==15);
    assert(remote_request(&motor,&p)==RES_OK);
    turn_begin(&turn,0,0);
    assert(remote_request(&motor,&p)==RES_DUPLICATE);
    p.angle_deg=-15; assert(remote_request(&motor,&p)==RES_STALE && motor.angle_deg==15);
    p.sequence=2; assert(remote_request(&motor,&p)==RES_BUSY && motor.angle_deg==15);
    /* 35mm on the powered right wheel, but 10mm on the passive left:
     * it is still below the 31.2mm differential goal, so do not stop. */
    turn.wheels.previous_a=(uint32_t)(10/STRAIGHT_MM_PER_COUNT);
    turn.wheels.previous_b=0U-(uint32_t)(35/STRAIGHT_MM_PER_COUNT);
    turn_step(&turn,&motor,(uint32_t)(10/STRAIGHT_MM_PER_COUNT),
        0U-(uint32_t)(35/STRAIGHT_MM_PER_COUNT));
    assert(motor.running && !motor.left_pwm);
    turn.wheels.previous_b=0U-(uint32_t)(45/STRAIGHT_MM_PER_COUNT);
    turn_step(&turn,&motor,(uint32_t)(10/STRAIGHT_MM_PER_COUNT),
        0U-(uint32_t)(45/STRAIGHT_MM_PER_COUNT));
    assert(!motor.running && motor.stop_reason==7);
    p.sequence=3; assert(remote_request(&motor,&p)==RES_OK);
    turn_begin(&turn,0,0);
    for(unsigned i=0;i<100;++i) {remote_step(&motor,false);turn_step(&turn,&motor,0,0);}
    assert(!motor.running && motor.stop_reason==4 && !motor.left_pwm && !motor.right_pwm);
    p.sequence=4; assert(remote_request(&motor,&p)==RES_OK);
    turn_begin(&turn,0,0);
    for(unsigned i=0;i<3;++i) {remote_step(&motor,true);turn_step(&turn,&motor,0,0);}
    assert(!motor.running && motor.stop_reason==2);
    for(unsigned i=0;i<3;++i) remote_step(&motor,false);
    p.sequence=5; assert(remote_request(&motor,&p)==RES_OK);
    turn_begin(&turn,0,0);
    for(unsigned i=0;i<500;++i) {
        if(i%10==0) remote_command(&motor,6+i,CMD_HEARTBEAT);
        remote_step(&motor,false);turn_step(&turn,&motor,0,0);
    }
    assert(!motor.running && motor.stop_reason==3 && !motor.left_pwm && !motor.right_pwm);
    const int sizes[]={1,3,5,15,30};
    for (int direction=-1;direction<=1;direction+=2) {
        double previous=0;
        for(unsigned i=0;i<sizeof(sizes)/sizeof(sizes[0]);++i) {
            double angle=simulate(direction*sizes[i],0.15,0xFFFFFF00U,false,0.08);
            assert(angle>previous); previous=angle;
        }
        simulate(direction*15,0,0,false,0.08);
        simulate(direction*5,0.15,0,true,0.08);
        /* Fast settling models the observed small coast without treating
         * encoder estimates as a measured physical yaw calibration. */
        double fast = simulate(direction*5,0.15,0,false,0.02);
        assert(fabs(fast-5.0) < 0.75);
    }
    puts("PASS: signed angles, passive-wheel rolling, size ordering, wrap, one stop and inherited stops");
}
