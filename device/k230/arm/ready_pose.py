# K230 UART1 transport for the existing user-selected ready pose.
# Host injects the checked config and the existing read-only UART1 helpers.
# Only writes SRAM goal/profile registers41..47 and torque-enable40=1.
from machine import UART, FPIOA
import time
import ujson

def frame(servo_id, instruction, params):
    body=bytes((servo_id,len(params)+2,instruction))+bytes(params)
    return b'\xff\xff'+body+bytes(((~sum(body))&255,))

def sync_frame(goals, profiles):
    if len(goals)!=6 or len(profiles)!=6:
        raise ValueError('Six axes required')
    params=[41,7]
    for index in range(6):
        goal=goals[index];profile=profiles[index]
        speed=profile['speed_raw'];acc=profile['acceleration_raw']
        if type(goal) is not int or not 0<=goal<=4095 or not 1<=speed<=1000 or not 1<=acc<=100:
            raise ValueError('Invalid servo goal/profile')
        params.extend((index+1,acc,goal&255,goal>>8,0,0,speed&255,speed>>8))
    return frame(254,0x83,params)

def read_checked(uart,i,address,size):
    data,error,reason=read_registers(uart,i,address,size)
    if data is None or error!=0:
        raise RuntimeError('ID%d register%d error=%s %s'%(i,address,error,reason))
    return data

def word(data,index):
    return data[index]|data[index+1]<<8

def feedback(uart,i):
    cmd=read_checked(uart,i,40,8)
    state=read_checked(uart,i,56,11)
    return {'id':i,'torque':cmd[0],'acceleration':cmd[1],'goal':word(cmd,2),
            'command_speed':word(cmd,6),'position':signed_position(state[0],state[1]),
            'speed':signed_position(state[2],state[3]),'moving':state[10]}

def snapshot(uart,event):
    rows=[feedback(uart,i) for i in range(1,7)]
    print('ARM_EVENT '+ujson.dumps({'event':event,'rows':rows}))
    return rows

def preflight(uart,config):
    regs=(21,22,23,24,25,26,27)
    for i in range(1,7):
        ident=read_checked(uart,i,3,4)
        settings=read_checked(uart,i,18,10)
        lim=read_checked(uart,i,9,4)
        if word(ident,0)!=777 or ident[2]!=i or ident[3]!=0:
            raise RuntimeError('ID%d identity mismatch'%i)
        if settings[0]!=12 or read_checked(uart,i,33,1)[0]!=0 or read_checked(uart,i,55,1)[0]!=1:
            raise RuntimeError('ID%d phase/mode/EEPROM lock mismatch'%i)
        for address in regs:
            if settings[address-18]!=config['registers'][i-1][str(address)]:
                raise RuntimeError('ID%d baseline register%d mismatch; no PID writes'%(i,address))
        low,high=word(lim,0),word(lim,2)
        current=feedback(uart,i)
        soft=config['software_limits'][i-1]
        goal=config['goals'][i-1]
        if not 0<=low<high<=4095 or not low<=goal<=high or not soft[0]<=goal<=soft[1]:
            raise RuntimeError('ID%d ready target outside limits'%i)
        if not low<=current['position']<=high or not soft[0]-2<=current['position']<=soft[1]+2:
            raise RuntimeError('ID%d position%d outside current device/software limits [%d,%d]'%(i,current['position'],low,high))
        if current['torque'] not in (0,1):
            raise RuntimeError('ID%d invalid torque state'%i)
    return snapshot(uart,'preflight_verified')

def send_sync(uart,goals,config):
    packet=sync_frame(goals,config['profiles'])
    if uart.write(packet)!=len(packet):
        raise RuntimeError('Short broadcast write')
    time.sleep_ms(2)
    # Sync-write has no ACK; independently read back all six targets/profiles.
    for i in range(1,7):
        row=feedback(uart,i);profile=config['profiles'][i-1]
        if row['goal']!=goals[i-1] or row['command_speed']!=profile['speed_raw'] or row['acceleration']!=profile['acceleration_raw']:
            raise RuntimeError('ID%d goal/profile readback mismatch'%i)
    print('ARM_EVENT '+ujson.dumps({'event':'sync_goal_verified','goals':goals,'packet_count':1}))

def enable_torque(uart,i):
    tx=frame(i,3,(40,1))
    for unused in range(8):
        if not uart.read():break
    if uart.write(tx)!=len(tx):raise RuntimeError('Short torque write')
    buffer=b'';started=time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(),started)<150:
        chunk=uart.read()
        if chunk:buffer+=chunk
        while True:
            packet,buffer=pop_packet(buffer)
            if packet is None:break
            if packet==tx or packet[2]!=i:continue
            if packet[3]!=2 or packet[4]!=0:
                raise RuntimeError('ID%d torque ACK error'%i)
            if read_checked(uart,i,40,1)[0]!=1:
                raise RuntimeError('ID%d torque readback failed'%i)
            return
        time.sleep_ms(1)
    raise RuntimeError('ID%d no torque ACK; no more commands'%i)

def initialize_hold(uart,config,rows):
    if any(row['torque']==0 for row in rows):
        hold=[]
        for i in range(1,7):
            row=feedback(uart,i);soft=config['software_limits'][i-1]
            goal=max(soft[0],min(soft[1],row['position']))
            if row['speed']!=0 or abs(goal-row['position'])>2:
                raise RuntimeError('ID%d current hold requires fresh stopped feedback'%i)
            hold.append(goal)
        # Set current goals while torque is OFF before enabling any axis.
        send_sync(uart,hold,config)
        for i in range(1,7):
            if read_checked(uart,i,40,1)[0]==0:enable_torque(uart,i)
    rows=snapshot(uart,'holding_current')
    if any(row['torque']!=1 for row in rows):raise RuntimeError('Six-axis torque not confirmed')
    print('ARM_TORQUE_ON: holding current position; remove supporting hands before ready dispatch')

def run_arm_ready(config,action):
    if action not in ('read','hold','ready'):raise ValueError('Unknown action')
    fpioa=FPIOA();fpioa.set_function(3,FPIOA.UART1_TXD);fpioa.set_function(4,FPIOA.UART1_RXD)
    uart=UART(UART.UART1,baudrate=1000000,bits=UART.EIGHTBITS,parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
    try:
        rows=preflight(uart,config)
        if action=='hold':initialize_hold(uart,config,rows)
        elif action=='ready':
            # A separate current-hold invocation and hands-free confirmation precede this action.
            if any(row['torque']!=1 for row in rows):raise RuntimeError('Run hold and confirm hands free before ready')
            send_sync(uart,config['goals'],config)
            print('ARM_READY_GOAL_SENT: torque remains ON; arrival requires observation')
            for unused in range(12):
                snapshot(uart,'ready_feedback');time.sleep_ms(250)
        print('ARM_ACTION_PASS '+action)
    finally:
        # Never issue recovery goals, torque-OFF, EEPROM or PID writes on failure.
        uart.deinit()


def run_ready_direct(config):
    """User-selected voice-ready dispatch: no servo reads or arrival wait.

    Preserve torque state. Rewriting torque-enable after the goal was observed
    to leave the goal register updated while the servo stayed at its old pose.
    Send only the final position/profile packet; do not re-arm or reset motion.
    """
    goal_packet=sync_frame(config['goals'],config['profiles'])
    pins=FPIOA();pins.set_function(3,FPIOA.UART1_TXD);pins.set_function(4,FPIOA.UART1_RXD)
    uart=UART(UART.UART1,baudrate=1000000,bits=UART.EIGHTBITS,parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
    try:
        started=time.ticks_ms()
        if uart.write(goal_packet)!=len(goal_packet):raise RuntimeError('Short ready goal write')
        # 56 bytes at 1Mbps take 0.56ms on the wire. Allow TX to finish before close.
        time.sleep_ms(2)
        print('ARM_EVENT '+ujson.dumps({'event':'ready_direct_sent','goals':config['goals'],
              'goal_packet_count':1,'torque_writes':0,'servo_reads':0,'readback_verified':False,
              'dispatch_ms':time.ticks_diff(time.ticks_ms(),started)}))
    finally:
        uart.deinit()
