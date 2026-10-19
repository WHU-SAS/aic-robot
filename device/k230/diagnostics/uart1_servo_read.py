# CanMV-K230-LP4 V3.0 / CanMV v1.8 / URT-2 / ST-3215-C018
# UART1 GPIO3(TX)->URT-2 TX, GPIO4(RX)->URT-2 RX; GND common.
# URT-2 logic powered at its 5V pin; level switch 3V3; USB unplugged.
# Servo power must be supplied separately. Remove the GPIO3/4 loopback jumper.
# Sends ONLY PING (0x01) and READ (0x02), never motion/torque/config writes.
# Open and run this file in CanMV IDE; do not save it as auto-start main.py.
from machine import UART, FPIOA
import time

IDS = (1, 2, 3, 4, 5, 6)
BAUD = 1000000
TIMEOUT_MS = 150
ATTEMPTS = 2


def make_packet(servo_id, instruction, params=()):
    if servo_id not in IDS or instruction not in (0x01, 0x02):
        raise ValueError("Only unicast PING/READ for IDs 1..6 is allowed")
    packet = bytearray((0xFF, 0xFF, servo_id, len(params) + 2, instruction))
    packet.extend(bytes(params))
    packet.append((~sum(packet[2:])) & 0xFF)
    return bytes(packet)


def pop_packet(buffer):
    # Immutable bytes avoid unsupported bytearray deletion in MicroPython.
    while len(buffer) >= 2:
        header = buffer.find(b"\xff\xff")
        if header < 0:
            return None, b"\xff" if buffer[-1] == 0xFF else b""
        buffer = buffer[header:]
        if len(buffer) < 4:
            return None, buffer
        if buffer[2] > 253 or buffer[3] < 2 or buffer[3] > 32:
            buffer = buffer[1:]
            continue
        total = buffer[3] + 4
        if len(buffer) < total:
            return None, buffer
        packet = buffer[:total]
        if (sum(packet[2:]) & 0xFF) != 0xFF:
            buffer = buffer[1:]
            continue
        return packet, buffer[total:]
    return None, buffer


def hex_bytes(data):
    return " ".join("%02X" % b for b in data) if data else "<empty>"


def request(uart, servo_id, instruction, params, expected_params):
    tx = make_packet(servo_id, instruction, params)
    captured = bytearray()
    echo_count = 0
    bad_shape = 0
    for attempt in range(ATTEMPTS):
        # Bounded drain so an old response cannot satisfy a new transaction.
        for unused in range(8):
            if not uart.read():
                break
        buffer = b""
        written = uart.write(tx)
        if written != len(tx):
            return None, "SHORT_TX %s/%s" % (written, len(tx))
        started = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), started) < TIMEOUT_MS:
            chunk = uart.read()
            if chunk:
                buffer += chunk
                if len(captured) < 96:
                    captured.extend(chunk[:96 - len(captured)])
            while True:
                packet, buffer = pop_packet(buffer)
                if packet is None:
                    break
                if packet == tx:
                    # Ping echo can look like a voltage-error status packet.
                    # Do not count it; the following READs are the real proof.
                    echo_count += 1
                    continue
                if packet[2] != servo_id:
                    continue
                count = packet[3] - 2
                if count == expected_params:
                    return packet, "OK"
                if packet[4] != 0 and count == 0:
                    return packet, "STATUS_ERROR 0x%02X (no data)" % packet[4]
                bad_shape += 1
            # No software-controlled direction pin is required by this setup.
            time.sleep_ms(1)
        time.sleep_ms(5)
    return None, "NO_REPLY echoes=%d bad_shape=%d RX=%s" % (
        echo_count, bad_shape, hex_bytes(captured))


def read_registers(uart, servo_id, address, size):
    packet, reason = request(uart, servo_id, 0x02, (address, size), size)
    if packet is None or packet[3] != size + 2:
        return None, packet[4] if packet else None, reason
    return packet[5:-1], packet[4], reason


def signed_position(lo, hi):
    raw = lo | (hi << 8)
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


def main():
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=BAUD, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    ping_ids = []
    read_ids = []
    clean_ids = []
    print("=== UART1 SERVO READ BEGIN ===")
    print("UART1 GPIO3/GPIO4; 1000000 baud, 8N1; IDs 1..6; PING/READ ONLY")
    try:
        for servo_id in IDS:
            print("--- ID %d ---" % servo_id)
            ping, reason = request(uart, servo_id, 0x01, (), 0)
            if ping is None:
                print("PING:", reason)
            else:
                ping_ids.append(servo_id)
                print("PING: OK STATUS=0x%02X" % ping[4])
            # STS registers 3..6: model low/high, configured ID, baud code.
            identity, id_error, id_reason = read_registers(uart, servo_id, 3, 4)
            # STS registers 56..63: position, speed, load, voltage, temperature.
            state, state_error, state_reason = read_registers(uart, servo_id, 56, 8)
            identity_ok = identity is not None and identity[2] == servo_id and identity[3] == 0
            if identity is None:
                print("IDENTITY:", id_reason)
            else:
                model = identity[0] | (identity[1] << 8)
                print("IDENTITY: MODEL_CODE=%d REG_ID=%d BAUD_CODE=%d STATUS=0x%02X" % (
                    model, identity[2], identity[3], id_error))
                if not identity_ok:
                    print("WARNING: configured ID or baud code differs from expected")
            if state is None:
                print("STATE:", state_reason)
            else:
                position = signed_position(state[0], state[1])
                print("STATE: POS=%d VOLTAGE=%.1fV TEMP=%dC STATUS=0x%02X" % (
                    position, state[6] / 10.0, state[7], state_error))
            if identity_ok and state is not None:
                read_ids.append(servo_id)
                print("ID %d READ_OK" % servo_id)
                if id_error == 0 and state_error == 0 and (ping is None or ping[4] == 0):
                    clean_ids.append(servo_id)
                else:
                    print("WARNING: servo reports a nonzero status; investigate before motion")
            else:
                print("ID %d READ_FAIL" % servo_id)
            time.sleep_ms(20)
    finally:
        uart.deinit()
    missing = [i for i in IDS if i not in read_ids]
    print("PING_RESPONDED:", ping_ids)
    print("READ_CONFIRMED:", read_ids)
    print("MISSING_OR_FAILED:", missing)
    print("STATUS_CLEAN:", clean_ids)
    print("RESULT: %d / 6 servos read" % len(read_ids))
    print("ALL_SERVOS_READ_OK" if not missing else "SERVO_READ_INCOMPLETE")
    print("=== UART1 SERVO READ END ===")


if __name__ == "__main__":
    main()
