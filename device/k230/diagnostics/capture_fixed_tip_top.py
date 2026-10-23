"""One fixed-jaw cube-top observation; UART sends PING/READ packets only."""
import os
import time
import ujson
import uhashlib
import binascii
from camera import CameraSession
from uart1_servo_read import FPIOA, UART, BAUD, read_registers, signed_position


def capture_fixed_tip_top(config, remote):
    record = {'sample_type':'fixed_fingertip_cube_top_contact', 'contacting_jaw':'fixed',
              'cube_edge_mm':20, 'samples_before':[], 'samples_after':[],
              'motion_commands_sent':False, 'pid_writes_sent':False,
              'torque_writes_sent':False, 'status':'incomplete'}
    camera = CameraSession(config)
    uart = None
    image = None

    def read_checked(sid, address, size):
        value, error, reason = read_registers(uart, sid, address, size)
        if value is None or error != 0:
            raise RuntimeError('ID%d READ%d error=%s %s' % (sid,address,error,reason))
        return value

    def sample():
        rows = {}
        for sid in range(1,7):
            started = time.ticks_ms()
            state = read_checked(sid,56,4)
            torque = read_checked(sid,40,1)
            rows[str(sid)] = {'position':signed_position(state[0],state[1]),
                'speed':signed_position(state[2],state[3]), 'torque':torque[0],
                'read_started_ticks_ms':started, 'read_finished_ticks_ms':time.ticks_ms()}
        return rows

    try:
        fpioa = FPIOA()
        fpioa.set_function(3,FPIOA.UART1_TXD)
        fpioa.set_function(4,FPIOA.UART1_RXD)
        uart = UART(UART.UART1,baudrate=BAUD,bits=UART.EIGHTBITS,
                    parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
        print('PAIR_STAGE camera_warmup')
        camera.open()
        record['clock']='K230 ticks_ms; before/after bracket, not hardware simultaneous'
        record['camera_settings']=config['camera']
        # AF getter is supported only before Sensor.run(); this OV5647 has no AF.
        # Do not set focus, servo torque, PID or target registers here.
        print('PAIR_STAGE encoders_before')
        for unused in range(3):
            record['samples_before'].append(sample())
            time.sleep_ms(50)
        record['snapshot_started_ticks_ms']=time.ticks_ms()
        image=camera.snapshot().copy()
        record['snapshot_finished_ticks_ms']=time.ticks_ms()
        print('PAIR_STAGE encoders_after')
        for unused in range(3):
            record['samples_after'].append(sample())
            time.sleep_ms(50)
        camera.stop()
        try:
            os.mkdir('/sdcard/formal_fixed_tip_top')
        except OSError:
            pass
        image.save(remote,quality=95)
        digest=uhashlib.sha256()
        size=0
        with open(remote,'rb') as source:
            while True:
                chunk=source.read(4096)
                if not chunk:
                    break
                digest.update(chunk)
                size+=len(chunk)
        record['camera_capture']={'remote':remote, 'size':size,
            'sha256':binascii.hexlify(digest.digest()).decode(),
            'width':config['camera']['width'], 'height':config['camera']['height'],
            'hmirror':config['camera']['hmirror'], 'vflip':config['camera']['vflip']}
        record['status']='raw_pair_saved_on_camera_sd'
    except BaseException as exc:
        record['error']=str(exc)
    finally:
        if uart is not None:
            uart.deinit()
        camera.close()
        print('PAIR_RECORD '+ujson.dumps(record))
        print('EXPORT_DONE')
