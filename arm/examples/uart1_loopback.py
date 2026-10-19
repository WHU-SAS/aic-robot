# CanMV-K230-LP4 V3.0 / CanMV v1.8
# JP1 physical pin 18 = GPIO3 = UART1 TX
# JP1 physical pin 13 = GPIO4 = UART1 RX
# Disconnect URT-2 and all external signals from these pins.
# Power OFF before adding/removing the jumper between pins 18 and 13.
from machine import UART, FPIOA
import time

fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
uart = UART(UART.UART1, baudrate=115200,
            bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE)
passed = 0
print("=== UART1 LOOPBACK: GPIO3 / GPIO4, 115200 8N1 ===")
try:
    for seq in range(10):
        uart.read()  # discard old bytes
        tx = b"\x00\x55\xaa\xffUART1_" + bytes([seq]) + b"\r\n"
        written = uart.write(tx)
        rx = bytearray()
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 200:
            chunk = uart.read()
            if chunk:
                rx.extend(chunk)
                if len(rx) >= len(tx):
                    break
            time.sleep_ms(2)
        ok = (written == len(tx) and bytes(rx) == tx)
        passed += int(ok)
        print("Round", seq + 1, "PASS" if ok else "FAIL",
              "TX=", tx, "RX=", bytes(rx))
        time.sleep_ms(100)
finally:
    uart.deinit()
print("RESULT:", passed, "/ 10")
print("UART1_LOOPBACK_PASS" if passed == 10 else "UART1_LOOPBACK_FAIL")