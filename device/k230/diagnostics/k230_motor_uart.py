# CanMV K230: UART2 IO5 TX -> PA09, IO6 RX <- PA08, GND common.
# Transient USBDBG bench test; does not overwrite SD main.py or control the arm.
from machine import UART, FPIOA
import time

MODE = "status"  # Moving modes require explicit host --execute and declared setup.
DRIVE = None  # Explicit (left_pwm, right_pwm, duration_ms), never a default move.

def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for unused in range(8):
            crc = (((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)) & 0xFFFF
    return crc

def encode(sequence, command, drive=None, speeds=None):
    suffix = command
    if command == 'DRIVE':
        if drive is None or len(drive) != 3 or any(type(v) is not int for v in drive):
            raise ValueError('DRIVE requires integer left/right PWM and duration ms')
        left, right, duration = drive
        if max(abs(left), abs(right)) > 1600 or not 10 <= duration <= 5000 or duration % 10:
            raise ValueError('DRIVE PWM must be within +/-1600; duration 10..5000 ms in 10 ms steps')
        suffix += ',%d,%d,%d' % (left, right, duration)
    elif drive is not None:
        raise ValueError('Wheel parameters require DRIVE')
    if command in ('FOLLOW', 'SPEED'):
        if speeds is None or len(speeds) != 2 or any(type(v) is not int or not 0 <= v <= 80 for v in speeds):
            raise ValueError('FOLLOW/SPEED require two integer wheel speeds within 0..80 mm/s')
        suffix += ',%d,%d' % tuple(speeds)
    elif speeds is not None:
        raise ValueError('Wheel speeds require FOLLOW or SPEED')
    body = ('M,%08X,%s' % (sequence, suffix)).encode()
    return b'@' + body + (',%04X\n' % crc16(body)).encode()

def decode(raw):
    # Resynchronize at the final '@' to tolerate startup/prefix noise.
    start = raw.rfind(b'@')
    if start < 0 or not raw.endswith(b'\n'):
        raise ValueError('No response frame')
    body, checksum = raw[start+1:-1].rsplit(b',', 1)
    if len(checksum) != 4 or int(checksum, 16) != crc16(body):
        raise ValueError('Response CRC mismatch')
    parts = body.decode().split(',')
    if len(parts) not in (11, 17) or parts[0] != 'R':
        raise ValueError('Response fields invalid')
    state = {'seq': int(parts[1], 16), 'command': parts[2], 'result': int(parts[3]),
             'ms': int(parts[4]), 'run': int(parts[5]), 'pwm': int(parts[6]),
             'encoder_a': int(parts[7]), 'encoder_b': int(parts[8]),
             'reason': int(parts[9]), 'last_seq': int(parts[10], 16)}
    if state['run'] not in (0, 1) or not 0 <= state['pwm'] <= 1600:
        raise ValueError('Invalid state range')
    state['protocol_version'] = 1
    if len(parts) == 17:
        state.update(protocol_version=int(parts[11]), left_pwm=int(parts[12]),
                     right_pwm=int(parts[13]), target_left=int(parts[14]),
                     target_right=int(parts[15]), remaining_ms=int(parts[16]))
        if state['protocol_version'] != 2 or not 0 <= state['remaining_ms'] <= 20000 or any(
                abs(state[k]) > 1600 for k in ('left_pwm', 'right_pwm', 'target_left', 'target_right')):
            raise ValueError('Invalid independent-wheel state')
    return state

class MotorLink:
    def __init__(self):
        fpioa = FPIOA()
        fpioa.set_function(5, FPIOA.UART2_TXD)
        fpioa.set_function(6, FPIOA.UART2_RXD)
        self.uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
                         parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        self.sequence = time.ticks_us() & 0xFFFFFFFF
        self.buffer = b''
        self.last = None

    def request(self, command, replay=None, drive=None, speeds=None):
        if replay is None:
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            sequence = self.sequence
        else:
            sequence = replay
        tx = encode(sequence, command, drive, speeds)
        if self.uart.write(tx) != len(tx):
            raise RuntimeError('Short UART write')
        started = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), started) < 600:
            chunk = self.uart.read()
            if chunk:
                self.buffer += chunk
                while b'\n' in self.buffer:
                    raw, self.buffer = self.buffer.split(b'\n', 1)
                    try:
                        state = decode(raw + b'\n')
                    except (ValueError, UnicodeError):
                        continue
                    if state['seq'] == sequence and state['command'] == command.split(',')[0]:
                        self.last = state
                        print('MOTOR_STATE', state)
                        return state
                if len(self.buffer) > 512:
                    self.buffer = self.buffer[-180:]
            time.sleep_ms(2)
        raise RuntimeError('No valid ACK for ' + command)

    def sync(self):
        state = self.request('STATUS')
        self.sequence = state['last_seq']
        return state

    def expect(self, command, result=0, replay=None, drive=None, speeds=None):
        state = self.request(command, replay, drive, speeds)
        if state['result'] != result:
            raise RuntimeError('Unexpected command result: ' + str(state))
        return state

    def drive(self, left_pwm, right_pwm, duration_ms):
        # Old firmware provides no capability marker: reject before any DRIVE
        # write instead of assuming an unacknowledged command is harmless.
        if self.last is None or self.last.get('protocol_version') != 2:
            raise RuntimeError('Independent-wheel control requires MSPM0 protocol v2; sync first')
        return self.expect('DRIVE', drive=(left_pwm, right_pwm, duration_ms))

    def stopped(self, reason):
        state = self.expect('STATUS')
        if state['run'] or state['pwm'] or state['reason'] != reason:
            raise RuntimeError('Stop verification failed: ' + str(state))
        return state

    def move(self, distance_mm):
        return self.expect('MOVE,%d' % distance_mm)

    def turn(self, angle_deg):
        return self.expect('TURN,%d' % angle_deg)

    def follow(self, left_mm_s, right_mm_s):
        # v2 framing alone does not imply FOLLOW support; deployment verifies
        # the installed image plus a stationary FOLLOW,0,0 / STOP exchange.
        return self.expect('FOLLOW', speeds=(left_mm_s, right_mm_s))

    def speed(self, left_mm_s, right_mm_s):
        return self.expect('SPEED', speeds=(left_mm_s, right_mm_s))

    def heartbeat_run(self, duration_ms):
        started = time.ticks_ms()
        maximum = 0
        while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
            state = self.expect('HB')
            maximum = max(maximum, state['pwm'])
            if not state['run']:
                raise RuntimeError('Motor stopped unexpectedly: ' + str(state))
            time.sleep_ms(200)
        return maximum

    def close(self):
        self.uart.deinit()

def run_bench(link):
    link.expect('STOP')
    link.stopped(1)
    link.uart.write(b'\x00partial@M,00000001,START,0000\n')
    time.sleep_ms(30)
    link.stopped(1)
    before = link.expect('STATUS')
    start = link.expect('START')
    if not start['run']:
        raise RuntimeError('START not applied')
    duplicate = link.expect('START', result=1, replay=start['seq'])
    if not duplicate['run']:
        raise RuntimeError('Duplicate START changed state')
    maximum = link.heartbeat_run(1800)
    link.expect('START', result=3)  # fresh START while running must not extend its deadline
    after = link.expect('STOP')
    link.stopped(1)
    if maximum != 1600 or not (after['encoder_a'] != before['encoder_a'] and
                              after['encoder_b'] != before['encoder_b']):
        raise RuntimeError('Ramp target or both encoder movements not observed')
    print('MOTOR_CHECK PASS command_stop_ramp_and_encoders')
    time.sleep_ms(300)
    start = link.expect('START')
    started = time.ticks_ms()
    # Continue heartbeats past 5 seconds; MCU must stop independently.
    while time.ticks_diff(time.ticks_ms(), started) < 5600:
        state = link.expect('HB')
        if not state['run']:
            if state['reason'] != 3 or state['pwm']:
                raise RuntimeError('Unexpected stop during run-limit test')
            break
        time.sleep_ms(200)
    else:
        raise RuntimeError('5-second motor limit did not stop output')
    duration = (state['ms'] - start['ms']) & 0xFFFFFFFF
    if not 4990 <= duration <= 5300:
        raise RuntimeError('Run-limit observed outside tolerance: ' + str(duration))
    link.stopped(3)
    print('MOTOR_CHECK PASS five_second_limit', duration)
    time.sleep_ms(300)
    start = link.expect('START')
    started = time.ticks_ms()
    # STATUS is deliberately not a heartbeat; receiver ISR owns the timeout.
    while time.ticks_diff(time.ticks_ms(), started) < 1600:
        time.sleep_ms(100)
        state = link.expect('STATUS')
        if not state['run']:
            break
    duration = (state['ms'] - start['ms']) & 0xFFFFFFFF
    if state['run'] or state['pwm'] or state['reason'] != 4 or not 990 <= duration <= 1300:
        raise RuntimeError('Heartbeat loss stop failed: ' + str(state))
    heartbeat = link.expect('HB')
    if heartbeat['run'] or heartbeat['pwm']:
        raise RuntimeError('Heartbeat restarted a stopped motor')
    print('MOTOR_CHECK PASS heartbeat_loss_no_restart', duration)
    link.expect('STOP')
    link.stopped(1)
    print('MOTOR_BENCH_PASS')

def run_button(link):
    link.expect('STOP')
    link.stopped(1)
    print('MOTOR_BUTTON_READY: 5 seconds until one start; press B21 after wheels turn')
    time.sleep_ms(5000)
    start = link.expect('START')
    if not start['run']:
        raise RuntimeError('START not applied')
    print('MOTOR_BUTTON_RUNNING: short press B21 once now')
    started = time.ticks_ms()
    maximum = 0
    while time.ticks_diff(time.ticks_ms(), started) < 5400:
        state = link.expect('HB')
        maximum = max(maximum, state['pwm'])
        if not state['run']:
            if state['reason'] != 2 or state['pwm']:
                raise RuntimeError('B21 stop not observed; no automatic retry: ' + str(state))
            break
        time.sleep_ms(100)
    else:
        raise RuntimeError('B21 stop not observed before run limit')
    if maximum == 0:
        raise RuntimeError('Stopped before any driven sample; repeat only after user review')
    elapsed = (state['ms'] - start['ms']) & 0xFFFFFFFF
    print('MOTOR_CHECK PASS button_stop', elapsed)
    # Keep sending heartbeats after the button press/release: neither may restart.
    stopped_at = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), stopped_at) < 2000:
        state = link.expect('HB')
        if state['run'] or state['pwm'] or state['reason'] != 2:
            raise RuntimeError('Stopped state changed after B21: ' + str(state))
        time.sleep_ms(100)
    link.stopped(2)
    print('MOTOR_CHECK PASS button_no_restart')
    link.expect('STOP')
    link.stopped(1)
    print('MOTOR_BUTTON_PASS')

def run_forward_once(link):
    link.expect('STOP')
    link.stopped(1)
    start = link.expect('START')
    if not start['run']:
        raise RuntimeError('START not applied')
    print('MOTOR_FORWARD_BEGIN')
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < 1000:
        state = link.expect('HB')
        if not state['run']:
            raise RuntimeError('Forward motion stopped unexpectedly: ' + str(state))
        time.sleep_ms(min(100, max(0, 1000-time.ticks_diff(time.ticks_ms(), started))))
    link.expect('STOP')
    link.stopped(1)
    # Include coast in the final encoder readout; do not reapply drive.
    stopped_at = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), stopped_at) < 2000:
        link.stopped(1)
        time.sleep_ms(100)
    print('MOTOR_FORWARD_PASS')

def run_drive_once(link, drive):
    encode(0, 'DRIVE', drive)  # Validate all parameters before sending commands.
    if link.last is None or link.last.get('protocol_version') != 2:
        raise RuntimeError('Independent-wheel control requires MSPM0 protocol v2')
    link.expect('STOP')
    link.stopped(1)
    start = link.drive(*drive)
    left, right, duration = drive
    if not left and not right:
        link.stopped(1)
        print('MOTOR_DRIVE_PASS')
        return
    if not start['run']:
        raise RuntimeError('DRIVE not applied')
    expected_reason = 3 if duration == 5000 else 5
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < duration + 1000:
        state = link.expect('HB')
        if not state['run']:
            if state['reason'] != expected_reason or state['pwm']:
                raise RuntimeError('Unexpected DRIVE stop: ' + str(state))
            break
        time.sleep_ms(100)
    else:
        raise RuntimeError('DRIVE did not stop at its requested duration')
    link.stopped(expected_reason)
    print('MOTOR_CHECK PASS independent_wheel_pulse', drive)
    print('MOTOR_DRIVE_PASS')

def main():
    link = MotorLink()
    failed = False
    try:
        link.sync()
        if MODE == 'bench':
            run_bench(link)
        elif MODE == 'forward':
            run_forward_once(link)
        elif MODE == 'button':
            run_button(link)
        elif MODE == 'drive':
            run_drive_once(link, DRIVE)
        elif MODE == 'stop':
            link.expect('STOP')
            link.stopped(1)
            print('MOTOR_STOP_PASS')
        else:
            for unused in range(20):
                state = link.expect('STATUS')
                if state['run'] or state['pwm']:
                    raise RuntimeError('Motor is already running; status mode sends no stop')
                time.sleep_ms(50)
            print('MOTOR_STATUS_PASS 20/20')
    except BaseException:
        failed = True
        raise
    finally:
        if MODE in ('bench', 'button', 'forward', 'drive') and failed:
            try:
                link.expect('STOP')
            except BaseException as exc:
                print('STOP_ACK_UNAVAILABLE; local 1s heartbeat limit remains active', exc)
        link.close()

if __name__ == '__main__':
    main()
