#include "follow_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static MotorParser request(uint32_t seq, MotorCommand cmd, unsigned left, unsigned right)
{
    return (MotorParser){.sequence=seq, .command=cmd, .left_speed=left, .right_speed=right};
}

static void parser_frames(void)
{
    const char *valid[]={"FOLLOW,0,0","FOLLOW,0,80","FOLLOW,80,0","SPEED,80,80"};
    const char *invalid[]={"FOLLOW,-1,0","FOLLOW,0,81","SPEED,81,0","SPEED,0,-1",
        "FOLLOW,1,1,1","FOLLOW,,0","SPEED,0,","FOLLOW,+1,0","SPEED,1.0,0"};
    MotorParser p={0};
    for(unsigned pass=0;pass<2;++pass){
        unsigned count=pass?sizeof(invalid)/sizeof(invalid[0]):sizeof(valid)/sizeof(valid[0]);
        for(unsigned i=0;i<count;++i){
            char body[48],frame[64];
            snprintf(body,sizeof(body),"M,%08X,%s",i+1,pass?invalid[i]:valid[i]);
            snprintf(frame,sizeof(frame),"@%s,%04X\n",body,motor_crc(body,strlen(body)));
            unsigned received=0;
            for(const char *c=frame;*c;++c)received+=motor_feed(&p,(uint8_t)*c);
            assert(received==(pass?0U:1U));
            if(!pass)assert(p.command==(i==3?CMD_SPEED:CMD_FOLLOW));
        }
    }
}

static void state_transitions(void)
{
    RemoteControl m={0}; FollowControl f;
    MotorParser p=request(1,CMD_SPEED,40,40);
    assert(remote_request(&m,&p)==RES_BUSY && !m.running);
    p=request(2,CMD_FOLLOW,40,80);
    assert(remote_request(&m,&p)==RES_OK && m.running && m.follow_mode);
    follow_begin(&f,0,0);
    for(unsigned i=0;i<60;++i){remote_step(&m,false);follow_step(&f,&m,0,0);}
    assert(m.run_ticks==60 && f.reference_left==40 && f.reference_right==80);
    int16_t pwm=m.left_pwm; float integral=f.integral_left;
    p=request(3,CMD_SPEED,80,40);
    assert(remote_request(&m,&p)==RES_OK && m.run_ticks==60 && m.lease_ticks==0);
    assert(m.left_pwm==pwm && f.integral_left==integral && f.reference_left==40);
    remote_step(&m,false);follow_step(&f,&m,0,0);
    assert(f.reference_left==41.5f && f.reference_right==78.5f);
    assert(abs(m.left_pwm-pwm)<=(int)REMOTE_PWM_STEP);
    assert(remote_request(&m,&p)==RES_DUPLICATE && m.run_ticks==61 && m.lease_ticks==1);
    p.left_speed=79;
    assert(remote_request(&m,&p)==RES_STALE && m.left_speed==80);
    p=request(4,CMD_FOLLOW,0,0);
    assert(remote_request(&m,&p)==RES_BUSY && m.run_ticks==61 && m.left_speed==80);
    p=request(5,CMD_MOVE,0,0);p.distance_mm=100;
    assert(remote_request(&m,&p)==RES_BUSY && m.follow_mode);
    p=request(6,CMD_TURN,0,0);p.angle_deg=5;
    assert(remote_request(&m,&p)==RES_BUSY && m.follow_mode);
    p=request(7,CMD_DRIVE,0,0);p.left_pwm=100;p.duration_ticks=50;
    assert(remote_request(&m,&p)==RES_BUSY && m.follow_mode);
    for(unsigned i=0;i<3;++i){remote_step(&m,true);follow_step(&f,&m,0,0);}
    assert(!m.running && !m.follow_mode && m.stop_reason==2 && !m.left_pwm && !m.right_pwm);
    p=request(8,CMD_SPEED,40,40);
    assert(remote_request(&m,&p)==RES_BUSY && !m.running);
    for(unsigned i=0;i<3;++i)remote_step(&m,false);
    assert(!m.running); /* Button release plus fresh SPEED cannot restart. */
    p.sequence=9;assert(remote_request(&m,&p)==RES_BUSY && !m.running);
    p=request(10,CMD_FOLLOW,0,0);assert(remote_request(&m,&p)==RES_OK);
    follow_begin(&f,0,0);
    for(unsigned i=0;i<100;++i){remote_step(&m,false);follow_step(&f,&m,0,0);}
    assert(!m.running && m.stop_reason==4 && !m.left_pwm && !m.right_pwm);
    p=request(11,CMD_SPEED,40,40);assert(remote_request(&m,&p)==RES_BUSY);
    p=request(12,CMD_FOLLOW,0,0);assert(remote_request(&m,&p)==RES_OK);
    follow_begin(&f,0,0);
    for(unsigned i=0;i<2000;++i){
        if(i%10==0){p=request(13+i,CMD_SPEED,0,0);assert(remote_request(&m,&p)==RES_OK);}
        remote_step(&m,false);follow_step(&f,&m,0,0);
        assert(!m.left_pwm && !m.right_pwm && !m.pwm);
        if(i==499)assert(m.running); /* Unlike legacy modes, continue past 5s. */
    }
    assert(!m.running && m.stop_reason==3 && m.run_ticks==2000);
    p=request(3000,CMD_SPEED,40,40);assert(remote_request(&m,&p)==RES_BUSY && !m.running);
    /* New FOLLOW is explicit; duplicate/stale FOLLOW after STOP cannot start. */
    p=request(3001,CMD_FOLLOW,30,30);assert(remote_request(&m,&p)==RES_OK);
    follow_begin(&f,0,0);remote_step(&m,false);follow_step(&f,&m,0,0);
    assert(remote_command(&m,2,CMD_STOP)==RES_STALE && !m.running);
    assert(remote_request(&m,&p)==RES_DUPLICATE && !m.running);
    p=request(3002,CMD_FOLLOW,81,0);assert(remote_request(&m,&p)==RES_INVALID);
    p=request(3003,CMD_MOVE,0,0);p.distance_mm=100;
    assert(remote_request(&m,&p)==RES_OK);
    p=request(3004,CMD_SPEED,40,40);assert(remote_request(&m,&p)==RES_BUSY && !m.follow_mode);
    p=request(3005,CMD_FOLLOW,40,40);assert(remote_request(&m,&p)==RES_BUSY && !m.follow_mode);
    remote_command(&m,3006,CMD_STOP);
    p=request(3007,CMD_FOLLOW,40,40);assert(remote_request(&m,&p)==RES_OK);
    follow_begin(&f,0,0);
    p=request(3008,CMD_SPEED,40,40);assert(remote_request(&m,&p)==RES_OK);
    for(unsigned i=0;i<100;++i){
        assert(remote_request(&m,&p)==RES_DUPLICATE);
        remote_step(&m,false);follow_step(&f,&m,0,0);
    }
    assert(!m.running && m.stop_reason==4); /* Replay cannot renew the lease. */
}

/* Synthetic motor responses verify feedback/ramp/arc behavior, not physical
 * yaw or actual chassis smoothness. Encoder wrap is exercised by the origin. */
static void model(uint32_t origin)
{
    RemoteControl m={0};FollowControl f;
    MotorParser p=request(1,CMD_FOLLOW,40,40);
    assert(remote_request(&m,&p)==RES_OK);follow_begin(&f,origin,origin);
    double left=0,right=0,vl=0,vr=0;
    unsigned seq=2;
    for(unsigned i=0;i<700;++i){
        unsigned l=i<200?40:(i<400?30:(i<550?60:0));
        unsigned r=i<200?40:(i<400?60:(i<550?30:0));
        if(i%10==0){p=request(seq++,CMD_SPEED,l,r);assert(remote_request(&m,&p)==RES_OK);}
        int16_t prev_l=m.left_pwm,prev_r=m.right_pwm;
        float prev_ref=f.reference_left;
        remote_step(&m,false);
        follow_step(&f,&m,origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT),
            origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT));
        assert(m.running && m.left_pwm>=0 && m.left_pwm<=1600 && m.right_pwm>=0 && m.right_pwm<=1600);
        assert(abs(m.left_pwm-prev_l)<=32 && abs(m.right_pwm-prev_r)<=32);
        assert(fabsf(f.reference_left-prev_ref)<=1.5001f);
        double target_l=fmax(0,m.left_pwm-900)*.20;
        double target_r=fmax(0,m.right_pwm-1000)*.20;
        vl+=(target_l-vl)*.01/.08;vr+=(target_r-vr)*.01/.08;
        left+=vl*.01;right+=vr*.01;
        if(i==199){assert(fabs(vl-40)<5 && fabs(vr-40)<5);}
        if(i==399){assert(fabs(vl-30)<5 && fabs(vr-60)<5);assert(right-left>30);}
        if(i==549){assert(fabs(vl-60)<5 && fabs(vr-30)<5);}
    }
    assert(m.running && m.run_ticks==700 && !m.left_pwm && !m.right_pwm);
    assert(f.reference_left==0 && f.reference_right==0 && !f.integral_left && !f.integral_right);
    assert(remote_command(&m,seq++,CMD_STOP)==RES_OK && !m.running);
    follow_step(&f,&m,origin,origin);
    assert(!m.left_pwm && !m.right_pwm);
    printf("follow model: %.2f/%.2fmm; smooth straight/left/right/zero, origin %08X\n",left,right,origin);
}

int main(void)
{
    parser_frames();state_transitions();model(0);model(0xFFFFFF00U);
    puts("PASS: continuous update, no restart, lease/B21/20s, zero, arc speed and wrap");
}
