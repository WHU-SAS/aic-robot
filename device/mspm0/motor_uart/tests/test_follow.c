#include "follow_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static MotorParser request(uint32_t seq, MotorCommand cmd, int16_t left, int16_t right)
{
    return (MotorParser){.sequence=seq, .command=cmd, .left_speed=left, .right_speed=right};
}

static void parser_frames(void)
{
    const char *valid[]={"FOLLOW,0,0","FOLLOW,0,80","FOLLOW,80,0","SPEED,80,80",
        "FOLLOW,-80,80","FOLLOW,80,-80","SPEED,-80,-80","SPEED,0,-1"};
    const int16_t left[]={0,0,80,80,-80,80,-80,0}, right[]={0,80,0,80,80,-80,-80,-1};
    const char *invalid[]={"FOLLOW,-81,0","FOLLOW,0,81","SPEED,81,0","SPEED,0,-81",
        "FOLLOW,1,1,1","FOLLOW,,0","SPEED,0,","FOLLOW,+1,0","SPEED,1.0,0",
        "FOLLOW,-32768,0","SPEED,0,65535","FOLLOW,--1,0","FOLLOW,-,0"};
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
            if(!pass){
                assert(p.command==(!strncmp(valid[i],"SPEED",5)?CMD_SPEED:CMD_FOLLOW));
                assert(p.left_speed==left[i] && p.right_speed==right[i]);
            }
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

/* Frozen pre-signed FOLLOW oracle. Only nonnegative commands reach this
 * function; compare every float bit and PWM over varied encoder/target traces. */
static int16_t forward_oracle_wheel(float target, float speed, float *reference,
                                    float *integral, int16_t previous)
{
    *reference += straight_clip(target - *reference,
        -STRAIGHT_ACCEL_MM_S2 * 0.01f, STRAIGHT_ACCEL_MM_S2 * 0.01f);
    if (*reference <= 0) {
        *integral = 0;
        return remote_ramp(previous, 0);
    }
    return straight_wheel(*reference, speed, integral, previous);
}

static void forward_oracle_step(FollowControl *s, RemoteControl *motor,
                                 uint32_t a, uint32_t b)
{
    float vl = (int32_t)(a - s->previous_a) * STRAIGHT_MM_PER_COUNT * 100.0f;
    float vr = -(int32_t)(b - s->previous_b) * STRAIGHT_MM_PER_COUNT * 100.0f;
    s->previous_a = a; s->previous_b = b;
    s->speed_left += 0.2f * (vl - s->speed_left);
    s->speed_right += 0.2f * (vr - s->speed_right);
    motor->left_pwm = forward_oracle_wheel(motor->left_speed, s->speed_left,
        &s->reference_left, &s->integral_left, motor->left_pwm);
    motor->right_pwm = forward_oracle_wheel(motor->right_speed, s->speed_right,
        &s->reference_right, &s->integral_right, motor->right_pwm);
    motor->target_left = motor->left_pwm; motor->target_right = motor->right_pwm;
    motor->pwm = motor->left_pwm > motor->right_pwm ? motor->left_pwm : motor->right_pwm;
}

static void positive_bitwise_regression(void)
{
    FollowControl actual, expected;
    RemoteControl m={.running=true,.follow_mode=true}, oracle=m;
    uint32_t a=0xFFFFFF00U,b=0x80U,rng=0x12345678U;
    follow_begin(&actual,a,b); follow_begin(&expected,a,b);
    for(unsigned i=0;i<10000;++i){
        rng=rng*1664525U+1013904223U;
        if(i%73==0){
            m.left_speed=oracle.left_speed=(int16_t)(rng%81);
            m.right_speed=oracle.right_speed=(int16_t)((rng>>8)%81);
        }
        if(i%503<70)m.left_speed=oracle.left_speed=0;
        if(i%719<70)m.right_speed=oracle.right_speed=0;
        a+=(uint32_t)((int)(rng%181)-25);
        b-=(uint32_t)((int)((rng>>16)%181)-25);
        follow_step(&actual,&m,a,b); forward_oracle_step(&expected,&oracle,a,b);
        assert(memcmp(&actual,&expected,sizeof(actual))==0);
        assert(m.left_pwm==oracle.left_pwm && m.right_pwm==oracle.right_pwm);
        assert(m.target_left==oracle.target_left && m.target_right==oracle.target_right);
        assert(m.pwm==oracle.pwm);
    }
    puts("positive FOLLOW: 10000 ticks bitwise equal to frozen original");
}

static void signed_replay_and_limits(void)
{
    RemoteControl m={0};
    MotorParser p=request(1,CMD_FOLLOW,-80,80);
    assert(remote_request(&m,&p)==RES_OK && m.left_speed==-80 && m.right_speed==80);
    assert(m.last_left_speed==-80 && m.last_right_speed==80);
    remote_step(&m,false);
    assert(remote_request(&m,&p)==RES_DUPLICATE && m.run_ticks==1 && m.lease_ticks==1);
    p.left_speed=80;
    assert(remote_request(&m,&p)==RES_STALE && m.left_speed==-80);
    p=request(2,CMD_SPEED,80,-80);
    assert(remote_request(&m,&p)==RES_OK && m.right_speed==-80 && m.lease_ticks==0);
    remote_step(&m,false);
    assert(remote_request(&m,&p)==RES_DUPLICATE && m.lease_ticks==1);
    p.right_speed=80;
    assert(remote_request(&m,&p)==RES_STALE && m.right_speed==-80);
    const int16_t invalid[]={-81,81,INT16_MIN,INT16_MAX};
    for(unsigned c=0;c<2;++c)for(unsigned side=0;side<2;++side)
        for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i){
            p=request(3,c?CMD_SPEED:CMD_FOLLOW,0,0);
            if(side)p.right_speed=invalid[i];else p.left_speed=invalid[i];
            assert(remote_request(&m,&p)==RES_INVALID && m.last_sequence==2);
            assert(m.left_speed==80 && m.right_speed==-80);
        }
    /* Same sequence with opposite signed payload must never match a replay. */
    assert(remote_command(&m,3,CMD_STOP)==RES_OK && !m.running);
    p=request(4,CMD_FOLLOW,-30,-30);
    assert(remote_request(&m,&p)==RES_OK);
    assert(remote_command(&m,1,CMD_STOP)==RES_STALE && !m.running);
    assert(remote_request(&m,&p)==RES_DUPLICATE && !m.running);
}

static void signed_wheel_symmetry(void)
{
    float pr=0,nr=0,pi=0,ni=0;int16_t pp=0,np=0;
    for(unsigned i=0;i<500;++i){
        float target=i<150?25:(i<300?80:0), speed=(float)((int)(i%107)-20);
        pp=follow_wheel(target,speed,&pr,&pi,pp);
        np=follow_wheel(-target,-speed,&nr,&ni,np);
        assert(np==-pp && nr==-pr && memcmp(&pi,&ni,sizeof(pi))==0);
        assert(abs(pp)<=REMOTE_PWM_TARGET && abs(np)<=REMOTE_PWM_TARGET);
    }
    assert(pp==0 && np==0 && pr==0 && nr==0 && pi==0 && ni==0);
}

static void signed_direction_transitions(void)
{
    FollowControl f;RemoteControl m={.running=true,.follow_mode=true,.left_speed=40,.right_speed=40};
    follow_begin(&f,0,0);
    for(unsigned i=0;i<100;++i)follow_step(&f,&m,0,0);
    assert(m.left_pwm>0 && m.right_pwm>0 && f.integral_left>0);
    /* 40 is not divisible by 1.5, so this exercises a crossing that skips zero. */
    m.left_speed=-40;
    bool left_zero=false,left_reverse=false;
    for(unsigned i=0;i<180;++i){
        int16_t pl=m.left_pwm,pr=m.right_pwm;float rr=f.reference_right,ir=f.integral_right;
        follow_step(&f,&m,0,0);
        assert(abs(m.left_pwm-pl)<=32 && abs(m.right_pwm-pr)<=32);
        assert(abs(m.left_pwm)<=1600 && abs(m.right_pwm)<=1600);
        int16_t expected_right=forward_oracle_wheel(40,0,&rr,&ir,pr);
        assert(f.reference_right==rr && f.integral_right==ir && m.right_pwm==expected_right);
        assert(m.right_pwm>0 && m.pwm==remote_magnitude(m.right_pwm));
        if(f.reference_left<0 && pl>0){
            assert(m.left_pwm>=0 && f.integral_left==0);
            assert(m.left_pwm==remote_ramp(pl,0));
        }
        if(m.left_pwm==0)left_zero=true;
        if(m.left_pwm<0){assert(left_zero);left_reverse=true;}
    }
    assert(left_reverse && m.left_pwm<0 && f.reference_left==-40);
    /* Reverse both independently, preserving each wheel's PWM slew bound. */
    m.left_speed=40;m.right_speed=-40;
    bool right_zero=false,right_reverse=false;
    for(unsigned i=0;i<180;++i){
        int16_t pl=m.left_pwm,pr=m.right_pwm;
        follow_step(&f,&m,0,0);
        assert(abs(m.left_pwm-pl)<=32 && abs(m.right_pwm-pr)<=32);
        if(pr==0)right_zero=true;
        if(m.right_pwm<0){assert(right_zero);right_reverse=true;}
        if(pl<0 && f.reference_left>0)assert(m.left_pwm<=0 && f.integral_left==0);
    }
    assert(right_reverse && m.left_pwm>0 && m.right_pwm<0);
    m.left_speed=0;m.right_speed=0;
    for(unsigned i=0;i<150;++i){
        int16_t pl=m.left_pwm,pr=m.right_pwm;
        follow_step(&f,&m,0,0);
        assert(abs(m.left_pwm-pl)<=32 && abs(m.right_pwm-pr)<=32);
    }
    assert(!m.left_pwm && !m.right_pwm && !m.pwm);
    assert(!f.reference_left && !f.reference_right && !f.integral_left && !f.integral_right);
    float ref=-3,integral=250;
    assert(follow_wheel(3,0,&ref,&integral,-64)==-96 && ref==-1.5f);
    assert(follow_wheel(3,0,&ref,&integral,-96)==-64 && ref==0 && integral==0);
    assert(follow_wheel(3,0,&ref,&integral,-64)==-32 && ref==1.5f && integral==0);
    assert(follow_wheel(3,0,&ref,&integral,-32)==0 && integral==0);
    assert(follow_wheel(3,0,&ref,&integral,0)==32);
}

static void signed_stop_protections(void)
{
    for(unsigned kind=0;kind<4;++kind){
        RemoteControl m={0};FollowControl f;
        MotorParser p=request(1,CMD_FOLLOW,-20,20);
        assert(remote_request(&m,&p)==RES_OK);follow_begin(&f,0,0);
        for(unsigned i=0;i<2000;++i){
            if(kind==3 && i%10==0){
                p=request(2+i,CMD_SPEED,-20,20);
                assert(remote_request(&m,&p)==RES_OK);
                assert(m.duration_ticks==2000 && m.run_ticks==i);
            }
            remote_step(&m,kind==1 && i>=30);follow_step(&f,&m,0,0);
            if(kind==0 && i==30){
                assert(remote_command(&m,2,CMD_STOP)==RES_OK);
                follow_step(&f,&m,0x12345678U,0x87654321U);
            }
            if(!m.running)break;
            assert(m.left_pwm<0 && m.right_pwm>0 && m.pwm<=1600);
        }
        const uint8_t reasons[]={1,2,4,3};
        assert(!m.running && !m.follow_mode && m.stop_reason==reasons[kind]);
        assert(!m.pwm && !m.left_pwm && !m.right_pwm && !m.left_speed && !m.right_speed);
        if(kind==2)assert(m.run_ticks==100);
        if(kind==3)assert(m.run_ticks==2000);
        p=request(5000,CMD_SPEED,-20,20);
        assert(remote_request(&m,&p)==RES_BUSY && !m.running);
    }
}

static void signed_feedback_model(uint32_t origin)
{
    RemoteControl m={0};FollowControl f;
    MotorParser p=request(1,CMD_FOLLOW,-20,20);
    assert(remote_request(&m,&p)==RES_OK);follow_begin(&f,origin,origin);
    double left=0,right=0,vl=0,vr=0;
    unsigned seq=2;
    for(unsigned i=0;i<900;++i){
        int16_t l=i<200?-20:(i<400?-40:(i<700?20:0));
        int16_t r=i<200?20:(i<400?0:(i<700?-20:0));
        if(i%10==0){p=request(seq++,CMD_SPEED,l,r);assert(remote_request(&m,&p)==RES_OK);}
        int16_t pl=m.left_pwm,pr=m.right_pwm;
        float rl=f.reference_left,rr=f.reference_right;
        remote_step(&m,false);
        follow_step(&f,&m,origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT),
            origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT));
        assert(m.running && abs(m.left_pwm)<=1600 && abs(m.right_pwm)<=1600 && m.pwm<=1600);
        assert(abs(m.left_pwm-pl)<=32 && abs(m.right_pwm-pr)<=32);
        assert(fabsf(f.reference_left-rl)<=1.5001f && fabsf(f.reference_right-rr)<=1.5001f);
        assert(!(pl>0 && m.left_pwm<0) && !(pl<0 && m.left_pwm>0));
        assert(!(pr>0 && m.right_pwm<0) && !(pr<0 && m.right_pwm>0));
        uint16_t al=remote_magnitude(m.left_pwm),ar=remote_magnitude(m.right_pwm);
        assert(m.pwm==(al>ar?al:ar));
        double tl=(m.left_pwm<0?-1:1)*fmax(0,abs(m.left_pwm)-900)*.20;
        double tr=(m.right_pwm<0?-1:1)*fmax(0,abs(m.right_pwm)-1000)*.20;
        vl+=(tl-vl)*.01/.08;vr+=(tr-vr)*.01/.08;
        left+=vl*.01;right+=vr*.01;
        if(i==199){assert(fabs(vl+20)<5 && fabs(vr-20)<5 && f.speed_left<0 && f.speed_right>0);}
        if(i==399){
            assert(fabs(vl+40)<5 && fabs(vr)<1 && m.left_pwm<0 && m.right_pwm==0);
            assert(f.reference_right==0 && f.integral_right==0);
        }
        if(i==699){assert(fabs(vl-20)<5 && fabs(vr+20)<5 && f.speed_left>0 && f.speed_right<0);}
    }
    assert(!m.left_pwm && !m.right_pwm && !m.pwm && fabs(vl)<1 && fabs(vr)<1);
    printf("signed model: %.2f/%.2fmm; spin, independent zero, reversal; origin %08X\n",left,right,origin);
}

int main(void)
{
    parser_frames();state_transitions();model(0);model(0xFFFFFF00U);
    positive_bitwise_regression();signed_replay_and_limits();signed_wheel_symmetry();
    signed_direction_transitions();signed_stop_protections();
    signed_feedback_model(0);signed_feedback_model(0xFFFFFF00U);
    puts("PASS: signed FOLLOW/SPEED, forward bitwise, independent zero crossing, replay, STOP/B21/lease/20s");
}
