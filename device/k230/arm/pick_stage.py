# Host injects the existing UART read/ready helpers and PositionSettled class.
# Executes one PC-selected action only. No next-stage goals are queued on board.

def run_pick_action(config,action,stage=None):
    if action not in ('read','hold','stage'):
        raise ValueError('Unknown pick action')
    fpioa=FPIOA()
    fpioa.set_function(3,FPIOA.UART1_TXD)
    fpioa.set_function(4,FPIOA.UART1_RXD)
    uart=UART(UART.UART1,baudrate=1000000,bits=UART.EIGHTBITS,
              parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
    try:
        rows=preflight(uart,config)
        if action=='hold':
            initialize_hold(uart,config,rows)
        elif action=='stage':
            if any(row['torque']!=1 for row in rows):
                raise RuntimeError('Six axes must hold torque before stage dispatch')
            send_sync(uart,config['goals'],config)
            print('PICK_EVENT '+ujson.dumps({'event':'stage_started','name':stage['name'],
                'goals':config['goals'],'packet_count':1}))
            started=time.ticks_ms()
            settled=PositionSettled(seconds=stage['stationary_s'],tolerance=stage['tolerance_counts']) if stage['name']=='close' else None
            while True:
                rows=[feedback(uart,i) for i in range(1,7)]
                elapsed=time.ticks_diff(time.ticks_ms(),started)/1000
                print('PICK_EVENT '+ujson.dumps({'event':'stage_feedback','name':stage['name'],
                    'elapsed_s':elapsed,'rows':rows}))
                if settled is not None:
                    if settled.update(rows[5]['position'],elapsed):
                        print('PICK_EVENT '+ujson.dumps({'event':'gripper_settled',
                            'details':settled.report(elapsed)}))
                        break
                elif elapsed>=stage['duration_s']:
                    break
                time.sleep_ms(50)
            print('PICK_EVENT '+ujson.dumps({'event':'stage_complete','name':stage['name'],'rows':rows}))
        print('PICK_ACTION_PASS '+action)
    finally:
        # Failure/interrupt leaves current goals and torque unchanged.
        uart.deinit()



def read_pick_positions(uart,goals):
    started=time.ticks_ms();positions=[]
    for servo_id in range(1,7):
        raw=read_checked(uart,servo_id,56,2)
        positions.append(signed_position(raw[0],raw[1]))
    return {'goals':list(goals),'positions':positions,
            'errors_actual_minus_goal':[a-b for a,b in zip(positions,goals)],
            'read_ms':time.ticks_diff(time.ticks_ms(),started)}


def classify_grip(goal,positions,threshold=30):
    """Apply the user-selected signed-delta rule to an already stable window."""
    if not positions:raise ValueError('Stable gripper positions required')
    if type(goal) is not int or any(type(value) is not int for value in positions):
        raise ValueError('Gripper goal and positions must be encoder counts')
    if type(threshold) is not int or threshold<0:raise ValueError('Invalid grip delta threshold')
    position=positions[-1];delta=position-goal
    return {'axis':6,'goal':goal,'position':position,'delta_counts':delta,
            'threshold_counts':threshold,'grasp_confirmed':delta>=threshold,
            'method':'axis6_stable_goal_delta','position_reference':'last_stable_sample'}


def run_pick_stage_direct(config,stage):
    """User-selected position gates and 200ms wrist lead; no torque/PID writes."""
    name=stage['name'];policy=stage['policy']
    if name not in ('hover','descend','close','lift'):raise ValueError('Unknown pick stage')
    # Validate and encode BOTH partial packets before sending the first one.
    packets=([sync_frame(config['goals'],config['profiles'],[4]),
              sync_frame(config['goals'],config['profiles'],[1,2,3,5,6])]
             if name=='descend' else [sync_frame(config['goals'],config['profiles'])])
    pins=FPIOA();pins.set_function(3,FPIOA.UART1_TXD);pins.set_function(4,FPIOA.UART1_RXD)
    uart=UART(UART.UART1,baudrate=1000000,bits=UART.EIGHTBITS,
              parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
    result=None;lead_ms=None
    try:
        if uart.write(packets[0])!=len(packets[0]):raise RuntimeError('Short pick goal write: first packet')
        if name=='descend':
            first_sent=time.ticks_ms()
            time.sleep_ms(policy['descend_axis4_lead_ms'])
            lead_ms=time.ticks_diff(time.ticks_ms(),first_sent)
            if uart.write(packets[1])!=len(packets[1]):raise RuntimeError('Short pick goal write: remaining axes')
        time.sleep_ms(2)
        print('PICK_EVENT '+ujson.dumps({'event':'stage_started','name':name,
              'goals':config['goals'],'packet_count':len(packets),'torque_writes':0,
              'dispatch_groups':[[4],[1,2,3,5,6]] if name=='descend' else [[1,2,3,4,5,6]],
              'axis4_lead_ms':lead_ms}))
        started=time.ticks_ms()
        if name in ('hover','descend'):
            tolerance=policy[name+'_tolerance_counts'];last_report=started
            while True:
                result=read_pick_positions(uart,config['goals'])
                if all(abs(error)<=tolerance for error in result['errors_actual_minus_goal']):
                    result.update({'tolerance_counts':tolerance,'source':'arrival_gate'})
                    print('PICK_EVENT '+ujson.dumps({'event':'position_gate_passed','name':name,'details':result}))
                    break
                now=time.ticks_ms()
                if time.ticks_diff(now,last_report)>=1000:
                    print('PICK_EVENT '+ujson.dumps({'event':'position_gate_waiting','name':name,
                          'tolerance_counts':tolerance,'errors_actual_minus_goal':result['errors_actual_minus_goal']}))
                    last_report=now
                time.sleep_ms(policy['poll_interval_ms'])
        elif name=='close':
            window=[];required=policy['gripper_stable_samples'];tolerance=policy['gripper_stable_tolerance_counts']
            while True:
                raw=read_checked(uart,6,56,2)
                window.append(signed_position(raw[0],raw[1]))
                if len(window)>required:window.pop(0)
                # A single +/-1 band across ALL five samples; no creeping drift.
                if len(window)==required and max(window)-min(window)<=2*tolerance:
                    print('PICK_EVENT '+ujson.dumps({'event':'gripper_settled','details':{
                        'sample_count':required,'position_min':min(window),'position_max':max(window),
                        'tolerance_counts':tolerance,'positions':list(window),
                        'elapsed_s':time.ticks_diff(time.ticks_ms(),started)/1000}}))
                    result=classify_grip(config['goals'][5],window,
                        policy.get('grip_confirmation_delta_counts',30))
                    print('PICK_EVENT '+ujson.dumps({'event':'grip_confirmation','details':result}))
                    break
                time.sleep_ms(policy['poll_interval_ms'])
        else:
            while time.ticks_diff(time.ticks_ms(),started)/1000<stage['duration_s']:
                time.sleep_ms(50)
        print('PICK_EVENT '+ujson.dumps({'event':'stage_complete','name':name,
              'arrival_checked':name in ('hover','descend')}))
        return result
    finally:
        uart.deinit()
