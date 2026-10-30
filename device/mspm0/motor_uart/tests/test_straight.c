#include "straight_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void parse_move(MotorParser *p, uint32_t sequence, int distance)
{
    char body[48], frame[64];
    snprintf(body, sizeof(body), "M,%08X,MOVE,%d", (unsigned)sequence, distance);
    snprintf(frame, sizeof(frame), "@%s,%04X\n", body, motor_crc(body, strlen(body)));
    unsigned complete = 0;
    for (const char *q = frame; *q; ++q) complete += motor_feed(p, (uint8_t)*q);
    assert(complete == 1 && p->command == CMD_MOVE && p->distance_mm == abs(distance));
    assert(p->reverse_move == (distance < 0));
}

static void simulate(float gain_left, float gain_right, float coast_s, uint32_t origin,
                     int distance, float dead_left, float dead_right)
{
    RemoteControl motor = {0}; StraightControl straight;
    MotorParser request = {0};
    parse_move(&request, 1, distance);
    assert(remote_request(&motor, &request) == RES_OK);
    straight_begin(&straight, origin, origin);
    double left=0, right=0, vl=0, vr=0, peak_difference=0;
    unsigned tick=0;
    for (; tick < 500 && motor.running; ++tick) {
        if (tick % 10 == 0) remote_command(&motor, 2+tick, CMD_HEARTBEAT);
        remote_step(&motor, false);
        straight_step(&straight, &motor,
            origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT),
            origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT));
        int direction=distance<0?-1:1;
        assert(direction*motor.left_pwm >= 0 && direction*motor.left_pwm <= 1600);
        assert(direction*motor.right_pwm >= 0 && direction*motor.right_pwm <= 1600);
        assert(motor.pwm==remote_magnitude(motor.left_pwm) || motor.pwm==remote_magnitude(motor.right_pwm));
        double target_l=direction*fmax(0, direction*motor.left_pwm-dead_left)*gain_left;
        double target_r=direction*fmax(0, direction*motor.right_pwm-dead_right)*gain_right;
        vl += (target_l-vl)*0.01/coast_s;
        vr += (target_r-vr)*0.01/coast_s;
        left+=vl*0.01; right+=vr*0.01;
        if(fabs(left-right)>peak_difference) peak_difference=fabs(left-right);
    }
    assert(!motor.running && motor.stop_reason == 6 && tick < 500);
    for (unsigned n=0; n<200; ++n) {
        vl += -vl*0.01/coast_s; vr += -vr*0.01/coast_s;
        left+=vl*0.01; right+=vr*0.01;
        remote_step(&motor, false);
        straight_step(&straight, &motor, origin, origin);
        assert(!motor.running && !motor.left_pwm && !motor.right_pwm);
    }
    double average=(left+right)/2;
    printf("model target=%d gain=%.2f/%.2f dead=%.0f/%.0f coast=%.2f: %.2fmm, wheel difference %.2fmm, peak %.2fmm, %ums\n",
        distance,gain_left,gain_right,dead_left,dead_right,coast_s,average,left-right,peak_difference,tick*10);
    /* Existing >=66mm models retain their 5% sanity limit. Short 30mm
     * exercises allow one configured finish-speed coast distance (3mm),
     * not a new physical short-distance accuracy claim. */
    double tolerance=fmax(abs(distance)*.05,STRAIGHT_FINISH_MM_S*STRAIGHT_COAST_S);
    assert(fabs(average-distance) <= tolerance && fabs(left-right) < 2);
}

int main(void)
{
    simulate(0.20f,0.18f,0.08f,0,150,900,1000);
    simulate(0.17f,0.16f,0.12f,0,150,900,1000);
    simulate(0.23f,0.20f,0.06f,0xFFFFFF00U,150,900,1000);
    /* Larger breakaway mismatch in either direction. These are stress
     * models, not motor parameters identified from the battery trials. */
    simulate(0.20f,0.20f,0.08f,0,100,900,600);
    simulate(0.20f,0.20f,0.08f,0,100,600,900);
    simulate(0.20f,0.20f,0.08f,0,66,900,600);
    /* Same closed loop in reverse, including unequal loads and counter wrap.
     * These are controller models, not physical reverse-distance acceptance. */
    simulate(0.20f,0.18f,0.08f,0,-150,900,1000);
    simulate(0.20f,0.18f,0.12f,0,-30,900,1000);
    simulate(0.20f,0.20f,0.08f,0,-30,900,600);
    simulate(0.20f,0.20f,0.08f,0xFFFFFF00U,-66,600,900);
    RemoteControl motor={0}; StraightControl straight; MotorParser p={0};
    parse_move(&p,1,150); assert(remote_request(&motor,&p)==RES_OK);
    straight_begin(&straight,0,0);
    assert(remote_request(&motor,&p)==RES_DUPLICATE);
    p.distance_mm=151; assert(remote_request(&motor,&p)==RES_STALE);
    p.sequence=2; assert(remote_request(&motor,&p)==RES_BUSY && motor.distance_mm==150);
    remote_command(&motor,3,CMD_STOP); straight_step(&straight,&motor,10,0);
    assert(!motor.running && !motor.left_pwm && !motor.right_pwm && !motor.distance_mm);
    p.sequence=4; p.distance_mm=150; assert(remote_request(&motor,&p)==RES_OK);
    straight_begin(&straight,0,0);
    for (unsigned i=0;i<100;++i) {remote_step(&motor,false);straight_step(&straight,&motor,0,0);}
    assert(!motor.running && motor.stop_reason==4);
    p.sequence=5; assert(remote_request(&motor,&p)==RES_OK);
    for(unsigned i=0;i<3;++i) {remote_step(&motor,true);straight_step(&straight,&motor,0,0);}
    assert(!motor.running && motor.stop_reason==2);
    for(unsigned i=0;i<3;++i)remote_step(&motor,false);
    parse_move(&p,6,-30);assert(remote_request(&motor,&p)==RES_OK);
    straight_begin(&straight,0,0);
    assert(remote_request(&motor,&p)==RES_DUPLICATE && motor.reverse_move);
    p.reverse_move=false;assert(remote_request(&motor,&p)==RES_STALE && motor.reverse_move);
    p.sequence=7;assert(remote_request(&motor,&p)==RES_BUSY && motor.reverse_move);
    assert(remote_command(&motor,8,CMD_STOP)==RES_OK && !motor.reverse_move);
    parse_move(&p,9,-30);assert(remote_request(&motor,&p)==RES_OK);
    straight_begin(&straight,0,0);
    for(unsigned i=0;i<500;++i){
        if(i%20==0)remote_command(&motor,10+i,CMD_HEARTBEAT);
        remote_step(&motor,false);straight_step(&straight,&motor,0,0);
    }
    assert(!motor.running && motor.stop_reason==3 && !motor.reverse_move);
    parse_move(&p,600,-30);assert(remote_request(&motor,&p)==RES_OK);
    straight_begin(&straight,0,0);
    for(unsigned i=0;i<100;++i){remote_step(&motor,false);straight_step(&straight,&motor,0,0);}
    assert(!motor.running && motor.stop_reason==4 && !motor.left_pwm && !motor.right_pwm);
    parse_move(&p,601,-30);assert(remote_request(&motor,&p)==RES_OK);
    straight_begin(&straight,0,0);
    for(unsigned i=0;i<3;++i){remote_step(&motor,true);straight_step(&straight,&motor,0,0);}
    assert(!motor.running && motor.stop_reason==2 && !motor.left_pwm && !motor.right_pwm);
    puts("PASS: distance models, asymmetric wheels, wrap, one stop, replay and inherited stops");
}
