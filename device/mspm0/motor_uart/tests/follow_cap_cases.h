/* Finite FOLLOW uses the production parser/state/controller; no hardware. */
static MotorParser cap_request(uint32_t seq, MotorCommand cmd, int16_t left,
                               int16_t right, uint16_t remaining)
{
    MotorParser p=request(seq,cmd,left,right);
    p.follow_bounded=true; p.follow_remaining_mm=remaining;
    return p;
}

/* Same accept-time hook as main.c, using the supplied encoder snapshot. */
static MotorResult cap_apply(RemoteControl *m, FollowControl *f, MotorParser *p,
                             uint32_t a, uint32_t b)
{
    MotorResult result=remote_request(m,p);
    if(result==RES_OK && p->command==CMD_FOLLOW && m->running)follow_begin(f,a,b);
    if(result==RES_OK && (p->command==CMD_FOLLOW || p->command==CMD_SPEED) &&
       p->follow_bounded && m->running)follow_set_limit(f,m,a,b);
    return result;
}

static void cap_parser_frames(void)
{
    const char *good[]={"FOLLOW,70,70,65535","SPEED,70,70,15","FOLLOW,0,0,0",
        "SPEED,0,0,0","FOLLOW,0,80,1","SPEED,80,0,0"};
    const uint16_t distances[]={65535,15,0,0,1,0};
    const char *bad[]={"SPEED,-1,1,10","FOLLOW,1,-1,10","SPEED,-0,0,1",
        "SPEED,1,1,-1","FOLLOW,1,1,65536","SPEED,1,1,1,2","FOLLOW,1,1,",
        "SPEED,1,1,+1","FOLLOW,81,0,1","SPEED,0,81,1","SPEED,1,1,1.0"};
    MotorParser parser={0};
    for(unsigned pass=0;pass<2;++pass){
        unsigned count=pass?sizeof(bad)/sizeof(bad[0]):sizeof(good)/sizeof(good[0]);
        for(unsigned i=0;i<count;++i){
            char body[64],frame[80];
            snprintf(body,sizeof(body),"M,%08X,%s",i+1,pass?bad[i]:good[i]);
            snprintf(frame,sizeof(frame),"@%s,%04X\n",body,motor_crc(body,strlen(body)));
            unsigned received=0;
            for(const char *c=frame;*c;++c)received+=motor_feed(&parser,(uint8_t)*c);
            assert(received==(pass?0U:1U));
            if(!pass)assert(parser.follow_bounded && parser.follow_remaining_mm==distances[i]);
        }
    }
}

static void cap_attach_replay_and_clear(void)
{
    RemoteControl m={0};FollowControl f;
    MotorParser p=request(1,CMD_FOLLOW,60,80);
    assert(cap_apply(&m,&f,&p,100,200)==RES_OK);
    f.speed_left=60;f.speed_right=80;f.reference_left=60;f.reference_right=80;
    f.integral_left=123;f.integral_right=234;
    m.left_pwm=900;m.right_pwm=1100;m.pwm=1100;m.run_ticks=73;m.lease_ticks=19;
    FollowControl prior=f;
    p=cap_request(2,CMD_SPEED,60,80,20);
    assert(cap_apply(&m,&f,&p,500,600)==RES_OK);
    assert(m.running && m.follow_cap_active && !m.follow_cap_completed && m.follow_remaining_mm==20);
    assert(m.run_ticks==73 && !m.lease_ticks && m.left_pwm==900 && m.right_pwm==1100);
    assert(f.cap_start_a==500 && f.cap_start_b==600);
    prior.cap_start_a=500;prior.cap_start_b=600;
    assert(memcmp(&f,&prior,sizeof(f))==0); /* No PI/filter/reference reset. */
    remote_step(&m,false);
    prior=f;
    assert(cap_apply(&m,&f,&p,900,200)==RES_DUPLICATE && m.lease_ticks==1);
    assert(memcmp(&f,&prior,sizeof(f))==0 && m.follow_remaining_mm==20);
    p.follow_remaining_mm=21;
    assert(cap_apply(&m,&f,&p,900,200)==RES_STALE && m.lease_ticks==1);
    assert(memcmp(&f,&prior,sizeof(f))==0 && m.follow_remaining_mm==20);
    p=cap_request(3,CMD_SPEED,60,80,30);
    assert(cap_apply(&m,&f,&p,900,200)==RES_OK && m.run_ticks==74);
    assert(f.cap_start_a==900 && f.cap_start_b==200);
    assert(f.reference_left==60 && f.reference_right==80 && f.integral_left==123);
    p=request(4,CMD_SPEED,-20,20);
    assert(cap_apply(&m,&f,&p,900,200)==RES_OK && !m.follow_cap_active);
    assert(!m.follow_cap_completed && !m.follow_remaining_mm && m.run_ticks==74);
    remote_step(&m,false);follow_step(&f,&m,10900,(uint32_t)(200U-10000U));
    assert(m.running && m.stop_reason==0); /* The old endpoint is gone. */
}

static void cap_zero_and_completed_state(void)
{
    RemoteControl m={0};FollowControl f;
    follow_begin(&f,100,200);f.integral_left=123;f.reference_left=30;
    FollowControl prior=f;
    MotorParser p=cap_request(1,CMD_FOLLOW,0,0,0);
    assert(cap_apply(&m,&f,&p,100,200)==RES_OK && !m.running && !m.pwm && m.stop_reason==6);
    assert(!m.follow_cap_completed && memcmp(&f,&prior,sizeof(f))==0);
    p=cap_request(2,CMD_SPEED,70,70,20);
    assert(cap_apply(&m,&f,&p,100,200)==RES_BUSY && !m.running);
    p=cap_request(3,CMD_SPEED,0,0,0);
    assert(cap_apply(&m,&f,&p,100,200)==RES_OK && !m.follow_cap_completed);
    p=cap_request(4,CMD_FOLLOW,70,70,10);
    assert(cap_apply(&m,&f,&p,100,200)==RES_OK && m.running);
    assert(f.cap_start_a==100 && f.cap_start_b==200);
    f.speed_left=f.speed_right=70;
    float remaining;
    uint32_t one=(uint32_t)lroundf(1.0f/STRAIGHT_MM_PER_COUNT);
    assert(!follow_cap_cutoff(&f,&m,100+one,200-one,&remaining));
    assert(remaining>8.9f && remaining<9.1f); /* D was not pre-debited for coast. */
    assert(follow_cap_cutoff(&f,&m,100+2*one,200-2*one,&remaining));
    assert(!m.running && !m.pwm && !m.follow_cap_active && m.follow_cap_completed && m.stop_reason==6);
    prior=f;
    p=cap_request(5,CMD_SPEED,70,70,200);
    assert(cap_apply(&m,&f,&p,900,900)==RES_OK && !m.running && m.follow_cap_completed);
    assert(memcmp(&f,&prior,sizeof(f))==0 && !m.pwm);
    p=request(6,CMD_SPEED,70,70);
    assert(cap_apply(&m,&f,&p,900,900)==RES_BUSY && m.follow_cap_completed);
    p=cap_request(7,CMD_SPEED,70,70,200);
    assert(cap_apply(&m,&f,&p,900,900)==RES_OK && !m.running);
    p=cap_request(8,CMD_FOLLOW,70,70,20);
    assert(cap_apply(&m,&f,&p,900,900)==RES_OK && m.running && !m.follow_cap_completed);
    assert(remote_command(&m,9,CMD_STOP)==RES_OK && !m.follow_cap_completed);
    p=cap_request(10,CMD_SPEED,70,70,20);
    assert(cap_apply(&m,&f,&p,900,900)==RES_BUSY);
    /* A plain MOVE reason6 is not a completed FOLLOW cap. */
    p=request(11,CMD_MOVE,0,0);p.distance_mm=100;
    assert(remote_request(&m,&p)==RES_OK);remote_stop(&m,6);
    assert(!m.follow_cap_completed);
    p=cap_request(12,CMD_SPEED,70,70,20);
    assert(cap_apply(&m,&f,&p,900,900)==RES_BUSY && !m.running);
}

static void cap_profile_and_existing_stops(void)
{
    assert(FOLLOW_CAP_CRUISE_MM_S==70 && FOLLOW_CAP_FINISH_MM_S==50 &&
           FOLLOW_CAP_SLOWDOWN_GAIN_PER_S==2 && STRAIGHT_COAST_S==0.12f);
    RemoteControl m={0};FollowControl f;
    MotorParser p=cap_request(1,CMD_FOLLOW,60,80,20);
    assert(cap_apply(&m,&f,&p,0,0)==RES_OK);
    f.reference_left=60;f.reference_right=80;
    for(unsigned i=0;i<20;++i){remote_step(&m,false);follow_step(&f,&m,0,0);}
    assert(m.running && f.reference_left==40 && f.reference_right==60); /* Mean50, differential20. */
    for(unsigned kind=0;kind<3;++kind){
        m=(RemoteControl){0};p=cap_request(1,CMD_FOLLOW,70,70,1);
        assert(cap_apply(&m,&f,&p,0,0)==RES_OK);
        if(kind==0)m.lease_ticks=99;
        if(kind==1){m.button_candidate=true;m.button_stable=true;m.button_ticks=3;}
        if(kind==2)m.run_ticks=1999;
        remote_step(&m,kind==1);follow_step(&f,&m,10000,(uint32_t)-10000);
        const uint8_t reason[]={4,2,3};
        assert(!m.running && m.stop_reason==reason[kind] && !m.follow_cap_completed);
        p=cap_request(2,CMD_SPEED,70,70,20);
        assert(cap_apply(&m,&f,&p,10000,(uint32_t)-10000)==RES_BUSY);
    }
}

static void cap_blocked_k230_model(uint32_t origin)
{
    RemoteControl m={0};FollowControl f;
    MotorParser p=request(1,CMD_FOLLOW,70,70);
    assert(cap_apply(&m,&f,&p,origin,origin)==RES_OK);
    double left=0,right=0,vl=0,vr=0;
    for(unsigned i=0;i<300;++i){
        if(i%20==0)assert(remote_command(&m,2+i,CMD_HEARTBEAT)==RES_OK);
        uint32_t a=origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT);
        uint32_t b=origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT);
        remote_step(&m,false);follow_step(&f,&m,a,b);
        double tl=fmax(0,m.left_pwm-900)*.20,tr=fmax(0,m.right_pwm-1000)*.20;
        vl+=(tl-vl)*.01/.12;vr+=(tr-vr)*.01/.12;left+=vl*.01;right+=vr*.01;
    }
    double start=(left+right)*.5;
    p=cap_request(400,CMD_SPEED,70,70,20);
    uint32_t a=origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT);
    uint32_t b=origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT);
    assert(cap_apply(&m,&f,&p,a,b)==RES_OK && m.running);
    unsigned stopped=0;
    /* No K230 callback/HB for780ms. The MCU still executes its distance. */
    for(unsigned i=0;i<78;++i){
        a=origin+(uint32_t)(int32_t)lround(left/STRAIGHT_MM_PER_COUNT);
        b=origin-(uint32_t)(int32_t)lround(right/STRAIGHT_MM_PER_COUNT);
        remote_step(&m,false);follow_step(&f,&m,a,b);
        if(!m.running && !stopped)stopped=i+1;
        double tl=m.running?fmax(0,m.left_pwm-900)*.20:0;
        double tr=m.running?fmax(0,m.right_pwm-1000)*.20:0;
        vl+=(tl-vl)*.01/.12;vr+=(tr-vr)*.01/.12;left+=vl*.01;right+=vr*.01;
    }
    double travel=(left+right)*.5-start;
    assert(stopped && stopped<78 && !m.running && m.stop_reason==6 && m.follow_cap_completed);
    assert(travel>18 && travel<22); /* Synthetic nominal plant only. */
    printf("bounded FOLLOW: stop at%ums, %.3fmm/20mm after780ms K230 gap, origin%08X\n",
           stopped*10,travel,origin);
}

static void finite_follow_cases(void)
{
    cap_parser_frames();cap_attach_replay_and_clear();cap_zero_and_completed_state();
    cap_profile_and_existing_stops();cap_blocked_k230_model(0);cap_blocked_k230_model(0xFFFFFF00U);
}
