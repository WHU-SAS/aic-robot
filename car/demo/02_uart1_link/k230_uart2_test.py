# CanMV-K230-LP4 V3.0/v1.8: GPIO5 TX -> MSPM0 PA09 RX;
# GPIO6 RX <- MSPM0 PA08 TX; GND common; separate power supplies.
# The MSPM0 must first run this directory's UART1 test firmware.
# Expected MSPM0_ACK is different from TX, so a local short is not a pass.
from machine import UART, FPIOA
import time

ROUNDS = 20
TIMEOUT_MS = 500

def main():
    fpioa = FPIOA()
    fpioa.set_function(5, FPIOA.UART2_TXD)
    fpioa.set_function(6, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    passed = 0
    print('=== UART2 MSPM0 LINK BEGIN ===')
    print('K230 UART2 GPIO5 TX/GPIO6 RX <-> MSPM0 UART1 PA09 RX/PA08 TX')
    print('115200 baud, 8N1; requires the MSPM0_UART1_TEST firmware')
    try:
        for seq in range(1, ROUNDS + 1):
            for unused in range(8):
                if not uart.read():
                    break
            token = '%02d,%08X' % (seq, time.ticks_us() & 0xFFFFFFFF)
            tx = ('K230_TEST,' + token + '\n').encode()
            expected = ('MSPM0_ACK,' + token + '\n').encode()
            written = uart.write(tx)
            buffer = b''
            captured = b''
            local_echo = False
            ok = False
            started = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), started) < TIMEOUT_MS:
                chunk = uart.read()
                if chunk:
                    buffer += chunk
                    captured = (captured + chunk)[:160]
                    while b'\n' in buffer:
                        end = buffer.find(b'\n') + 1
                        frame, buffer = buffer[:end], buffer[end:]
                        if frame == tx:
                            local_echo = True
                        if frame == expected and written == len(tx):
                            ok = True
                            break
                    if ok:
                        break
                    if len(buffer) > 160:
                        buffer = buffer[-80:]
                time.sleep_ms(2)
            passed += int(ok)
            print('Round %02d %s TX=%r RX=%r' % (
                seq, 'PASS' if ok else 'FAIL', tx, captured))
            if not ok and local_echo:
                print('LOCAL_ECHO_ONLY: expected MSPM0_ACK, not a copy of K230_TEST')
            time.sleep_ms(50)
    finally:
        uart.deinit()
    print('RESULT: %d / %d' % (passed, ROUNDS))
    print('UART2_MSPM0_LINK_PASS' if passed == ROUNDS else 'UART2_MSPM0_LINK_FAIL')
    print('=== UART2 MSPM0 LINK END ===')

if __name__ == '__main__':
    main()
