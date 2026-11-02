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

typedef struct { float gain_l,gain_r,dead_l,dead_r,tau; } ShortPlant;
static double short_max_error, short_max_difference, short_max_passive;
static unsigned short_cases, arc_cases, short_max_ms;
static double long_arc_max_error, long_arc_max_passive;
static unsigned long_arc_cases, long_arc_max_ms;

static void recorded_arc_micro_motion_cannot_seed_negative_integral(void)
{
    /* 2026-11-03 ARC,-5,5 MCU samples, relative to its accepted command.
     * Interpolate the ~60ms samples onto 10ms control ticks. This regression
     * replays measured counts, not a reconstruction of unsampled wheel motion. */
    static const struct { unsigned ms; int a,b; } samples[]={
        {0,0,0},{60,-2,0},{120,-19,-2},{190,-26,-12},{250,-23,-30},
        {310,-77,-30},{380,-78,-30},{440,-69,-30},{500,-64,-30},
        {570,-64,-30},{630,-64,-30}};
    RemoteControl motor={0};StraightControl control;
    MotorParser request={.sequence=1,.command=CMD_ARC,.arc_left_mm=-5,.arc_right_mm=5};
    assert(remote_request(&motor,&request)==RES_OK);
    straight_short_begin(&control,0,0,-5,5);
    unsigned index=0;
    int16_t left_at_500=0,right_at_500=0;
    for(unsigned tick=1;tick<=63;++tick) {
        unsigned ms=tick*10;
        while(index+1<sizeof(samples)/sizeof(*samples)-1 && samples[index+1].ms<ms)++index;
        float fraction=(float)(ms-samples[index].ms)/(samples[index+1].ms-samples[index].ms);
        int32_t a=(int32_t)lroundf(samples[index].a+fraction*(samples[index+1].a-samples[index].a));
        int32_t b=(int32_t)lroundf(samples[index].b+fraction*(samples[index+1].b-samples[index].b));
        if(tick%10==0)remote_command(&motor,tick+2,CMD_HEARTBEAT);
        int16_t previous_left=motor.left_pwm,previous_right=motor.right_pwm;
        bool left_started=control.short_left_started,right_started=control.short_right_started;
        remote_step(&motor,false);
        straight_short_step(&control,&motor,(uint32_t)a,(uint32_t)b,8);
        if(!left_started&&control.short_left_started)assert(control.integral_left>=0);
        if(!right_started&&control.short_right_started)assert(control.integral_right>=0);
        assert(abs(motor.left_pwm-previous_left)<=32 && abs(motor.right_pwm-previous_right)<=32);
        assert(motor.left_pwm<=0 && motor.left_pwm>=-1600);
        assert(motor.right_pwm>=0 && motor.right_pwm<=1600);
        if(tick==50) {left_at_500=motor.left_pwm;right_at_500=motor.right_pwm;}
    }
    fprintf(stderr,"recorded ARC micro-motion: started=%d/%d PWM=%d/%d\n",
        control.short_left_started,control.short_right_started,motor.left_pwm,motor.right_pwm);
    assert(motor.running && control.short_left_started && control.short_right_started);
    assert(motor.left_pwm<=-1100 && motor.right_pwm>=1100);
    assert(motor.left_pwm<left_at_500 && motor.right_pwm>right_at_500);
    remote_command(&motor,1000,CMD_STOP);
    straight_short_step(&control,&motor,0,0,8);
    assert(!motor.running && !motor.left_pwm && !motor.right_pwm);
}

/* These varied gain/dead-zone/inertia plants are not identified vehicle
 * parameters. They exercise tiny targets that the former >=30mm tests missed. */
static void simulate_short(ShortPlant plant, int left_goal, int right_goal,
                           bool arc, uint32_t origin)
{
    RemoteControl motor={0}; StraightControl control; MotorParser request={0};
    request.sequence=1;
    if(arc) {
        request.command=CMD_ARC;
        request.arc_left_mm=left_goal; request.arc_right_mm=right_goal;
    } else {
        assert(left_goal==right_goal);
        request.command=CMD_MOVE; request.distance_mm=abs(left_goal);
        request.reverse_move=left_goal<0;
    }
    assert(remote_request(&motor,&request)==RES_OK);
    if(arc) straight_short_begin(&control,origin,origin,left_goal,right_goal);
    else straight_begin(&control,origin,origin);
    double left=0,right=0,vl=0,vr=0;
    bool left_latched=false,right_latched=false;
    unsigned ticks=0;
    for(;ticks<500&&motor.running;++ticks) {
        if(ticks%10==0)remote_command(&motor,ticks+2,CMD_HEARTBEAT);
        remote_step(&motor,false);
        uint32_t a=origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT);
        uint32_t b=origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT);
        if(arc)straight_short_step(&control,&motor,a,b,8);
        else straight_step(&control,&motor,a,b);
        assert((left_goal<0 ? -motor.left_pwm : motor.left_pwm)>=0);
        assert((right_goal<0 ? -motor.right_pwm : motor.right_pwm)>=0);
        assert(remote_magnitude(motor.left_pwm)<=REMOTE_PWM_TARGET);
        assert(remote_magnitude(motor.right_pwm)<=REMOTE_PWM_TARGET);
        if(!left_goal||left_latched)assert(!motor.left_pwm);
        if(!right_goal||right_latched)assert(!motor.right_pwm);
        left_latched=left_latched||control.left_done;
        right_latched=right_latched||control.right_done;
        if(left_latched)assert(!motor.left_pwm);
        if(right_latched)assert(!motor.right_pwm);
        double target_l=(motor.left_pwm<0 ? -1:1)*
            fmax(0,remote_magnitude(motor.left_pwm)-plant.dead_l)*plant.gain_l;
        double target_r=(motor.right_pwm<0 ? -1:1)*
            fmax(0,remote_magnitude(motor.right_pwm)-plant.dead_r)*plant.gain_r;
        /* A zero-PWM passive wheel can roll while the other drives. */
        if(!left_goal)target_l+=.03*vr;
        if(!right_goal)target_r+=.03*vl;
        vl+=(target_l-vl)*.01/plant.tau;
        vr+=(target_r-vr)*.01/plant.tau;
        left+=vl*.01;right+=vr*.01;
    }
    if(motor.running||motor.stop_reason!=(arc?8:6)||ticks>=500)
        fprintf(stderr,"tiny completion goals=%d/%d actual=%.3f/%.3f dead=%.0f/%.0f tau=%.2f ticks=%u reason=%u\n",
            left_goal,right_goal,left,right,plant.dead_l,plant.dead_r,plant.tau,ticks,motor.stop_reason);
    assert(!motor.running&&motor.stop_reason==(arc?8:6)&&ticks<500);
    assert(left_latched&&right_latched);
    for(unsigned tick=0;tick<200;++tick) {
        vl+=-vl*.01/plant.tau;vr+=-vr*.01/plant.tau;
        left+=vl*.01;right+=vr*.01;
        remote_step(&motor,false);
        if(arc)straight_short_step(&control,&motor,origin,origin,8);
        else straight_step(&control,&motor,origin,origin);
        assert(!motor.running&&!motor.left_pwm&&!motor.right_pwm);
    }
    double error=fmax(fabs(left-left_goal),fabs(right-right_goal));
    bool long_arc=arc&&(abs(left_goal)>STRAIGHT_SHORT_LIMIT_MM||abs(right_goal)>STRAIGHT_SHORT_LIMIT_MM);
    if(long_arc) {
        double active_error=fmax(left_goal?fabs(left-left_goal):0,right_goal?fabs(right-right_goal):0);
        double passive=fmax(left_goal?0:fabs(left),right_goal?0:fabs(right));
        if(active_error>long_arc_max_error)long_arc_max_error=active_error;
        if(!left_goal&&fabs(left)>long_arc_max_passive)long_arc_max_passive=fabs(left);
        if(!right_goal&&fabs(right)>long_arc_max_passive)long_arc_max_passive=fabs(right);
        if(ticks*10>long_arc_max_ms)long_arc_max_ms=ticks*10;
        ++long_arc_cases;
        /* Larger arcs retain the 5s deadline and the 1.5mm driven-wheel
         * model bound; passive travel is separately bounded by 3% coupling.
         * Neither bound is a physical accuracy claim. */
        if(active_error>1.5)fprintf(stderr,"long ARC goals=%d/%d actual=%.3f/%.3f tau=%.2f\n",
            left_goal,right_goal,left,right,plant.tau);
        assert(active_error<=1.5);
        assert(passive<=.031*fmax(abs(left_goal),abs(right_goal)));
        return;
    }
    if(error>short_max_error)short_max_error=error;
    if(!arc&&fabs(left-right)>short_max_difference)short_max_difference=fabs(left-right);
    if(!left_goal&&fabs(left)>short_max_passive)short_max_passive=fabs(left);
    if(!right_goal&&fabs(right)>short_max_passive)short_max_passive=fabs(right);
    if(ticks*10>short_max_ms)short_max_ms=ticks*10;
    /* Accepted startup tradeoff: preventing negative preload raises these
     * synthetic maxima from0.731/0.693mm (short/long) to1.420/1.292mm.
     * Allow1.5mm here; the physical parking window still requires real tests. */
    if(error>1.5)fprintf(stderr,"tiny model goals=%d/%d actual=%.3f/%.3f dead=%.0f/%.0f tau=%.2f\n",
        left_goal,right_goal,left,right,plant.dead_l,plant.dead_r,plant.tau);
    assert(error<=1.5); /* Model regression target, not physical acceptance. */
    if(arc)++arc_cases;else ++short_cases;
}

int main(void)
{
    recorded_arc_micro_motion_cannot_seed_negative_integral();
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
    const ShortPlant plants[]={
        {.20f,.18f,900,1000,.03f},{.20f,.18f,900,1000,.06f},
        {.20f,.18f,900,1000,.08f},{.17f,.16f,900,1000,.12f},
        {.23f,.20f,900,1000,.06f},{.20f,.20f,900,600,.08f},
        {.20f,.20f,600,900,.08f},{.20f,.18f,1000,1050,.08f},
        {.20f,.18f,1050,1000,.08f},{.20f,.18f,1100,1000,.08f}};
    const int lengths[]={3,6,7,12,20,30};
    const int arcs[][2]={{2,9},{9,2},{0,9},{9,0},{3,-5},{1,9},{9,1}};
    const int long_arcs[][2]={{31,31},{31,-31},{0,31},{60,60},{60,-60},{60,0},
        {100,100},{100,-100},{0,100},{1,63},{2,64},{38,100},{0,62},
        {31,1},{63,1},{64,2},{100,38},{62,0},{100,-38}};
    for(unsigned model=0;model<sizeof(plants)/sizeof(*plants);++model)
        for(int direction=-1;direction<=1;direction+=2) {
            for(unsigned goal=0;goal<sizeof(lengths)/sizeof(*lengths);++goal)
                simulate_short(plants[model],direction*lengths[goal],direction*lengths[goal],
                    false,model%2?0xFFFFFF00U:0);
            for(unsigned goal=0;goal<sizeof(arcs)/sizeof(*arcs);++goal)
                simulate_short(plants[model],direction*arcs[goal][0],direction*arcs[goal][1],
                    true,model%2?0xFFFFFF00U:0);
            for(unsigned goal=0;goal<sizeof(long_arcs)/sizeof(*long_arcs);++goal)
                simulate_short(plants[model],direction*long_arcs[goal][0],direction*long_arcs[goal][1],
                    true,model%2?0xFFFFFF00U:0);
        }
    printf("tiny/ARC model cases=%u/%u max wheel error=%.3fmm, straight difference=%.3fmm, passive roll=%.3fmm, max=%ums\n",
        short_cases,arc_cases,short_max_error,short_max_difference,short_max_passive,short_max_ms);
    printf("long ARC models=%u max driven-wheel error=%.3fmm, passive roll=%.3fmm, max=%ums\n",
        long_arc_cases,long_arc_max_error,long_arc_max_passive,long_arc_max_ms);
    puts("PASS: distance models, asymmetric wheels, wrap, one stop, replay and inherited stops");
}
