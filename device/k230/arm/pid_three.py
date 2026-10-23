"""Fixed PID trial; host supplies UART read helpers, sync_frame and CONFIG."""
import json


def emit(event, **fields):
    fields['event'] = event
    print('PID_SAMPLE ' + json.dumps(fields))


def run_trial(config):
    pins = FPIOA()
    pins.set_function(3, FPIOA.UART1_TXD)
    pins.set_function(4, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=1000000, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    try:
        for index, stage in enumerate(config['stages']):
            packet = sync_frame(stage['goals'], config['profiles'])
            if uart.write(packet) != len(packet):
                raise RuntimeError('Incomplete goal packet')
            started = time.ticks_ms()
            emit('stage_started', index=index, name=stage['name'], goals=stage['goals'])
            while time.ticks_diff(time.ticks_ms(), started) < stage['seconds'] * 1000:
                cycle = time.ticks_ms()
                rows = []
                for servo_id in range(1, 7):
                    raw, error, reason = read_registers(uart, servo_id, 56, 4)
                    elapsed = time.ticks_diff(time.ticks_ms(), started) / 1000
                    if raw is None or error:
                        rows.append([elapsed, None, None, str(error) + ':' + reason])
                    else:
                        rows.append([elapsed, signed_position(raw[0], raw[1]),
                                     signed_position(raw[2], raw[3]), None])
                emit('sample', index=index, rows=rows)
                wait = config['sample_period_ms'] - time.ticks_diff(time.ticks_ms(), cycle)
                if wait > 0:
                    time.sleep_ms(wait)
            emit('stage_complete', index=index, name=stage['name'])
        # Record the actual settings once, after the timed movement sequence.
        parameters = []
        for servo_id in range(1, 7):
            raw, error, reason = read_registers(uart, servo_id, 21, 7)
            parameters.append({'id':servo_id, 'registers_21_to_27':list(raw) if raw is not None else None,
                               'error':error, 'reason':reason})
        emit('parameters', rows=parameters)
        emit('complete')
    finally:
        uart.deinit()
