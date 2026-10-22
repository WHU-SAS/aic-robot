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
