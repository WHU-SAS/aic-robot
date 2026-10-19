"""Feetech STS servo access in raw encoder counts.

No joint-angle conversion is defined here. Physical joint calibration and
software direction/zero offsets must be provided before using angle commands.
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)

import threading
import time

from scservo_sdk import (
    COMM_SUCCESS,
    PortHandler,
    SMS_STS_GOAL_POSITION_L,
    SMS_STS_MAX_ANGLE_LIMIT_L,
    SMS_STS_MIN_ANGLE_LIMIT_L,
    SMS_STS_MODE,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_LOAD_L,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_VOLTAGE,
    SMS_STS_TORQUE_ENABLE,
    sms_sts,
)


class ServoError(RuntimeError):
    """A servo did not respond or rejected an operation."""


class ServoBus:
    def __init__(self, port: str, baud: int = 1_000_000):
        self.port_name = port
        self.port = PortHandler(port)
        self.port.baudrate = baud
        self.lock = threading.RLock()
        try:
            if not self.port.openPort():
                raise ServoError(f"Cannot open {port} at {baud} bps")
        except Exception:
            if self.port.is_open:
                self.port.closePort()
            raise
        self.sdk = sms_sts(self.port)

    def close(self) -> None:
        with self.lock:
            if self.port.is_open:
                self.port.closePort()

    def __enter__(self) -> "ServoBus":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _value(self, response, operation: str, servo_id: int) -> int:
        value, result, error = response
        if result != COMM_SUCCESS or error:
            raise ServoError(
                f"ID {servo_id} {operation}: {self.sdk.getTxRxResult(result)} "
                f"(servo error {error})"
            )
        return value

    def _written(self, response, operation: str, servo_id: int) -> None:
        result, error = response
        if result != COMM_SUCCESS or error:
            raise ServoError(
                f"ID {servo_id} {operation}: {self.sdk.getTxRxResult(result)} "
                f"(servo error {error})"
            )

    @staticmethod
    def _check_id(servo_id: int) -> None:
        if type(servo_id) is not int or not 0 <= servo_id <= 253:
            raise ValueError("servo ID must be an integer from 0 to 253")

    def ping(self, servo_id: int) -> int:
        self._check_id(servo_id)
        with self.lock:
            return self._value(self.sdk.ping(servo_id), "ping", servo_id)

    def read_register(self, servo_id: int, address: int, size: int = 1) -> int:
        self._check_id(servo_id)
        with self.lock:
            if size == 1:
                response = self.sdk.read1ByteTxRx(servo_id, address)
            elif size == 2:
                response = self.sdk.read2ByteTxRx(servo_id, address)
            else:
                raise ValueError("register size must be 1 or 2 bytes")
            return self._value(response, f"read register {address}", servo_id)

    def read_position_speed(self, servo_id: int) -> tuple[int, int]:
        self._check_id(servo_id)
        with self.lock:
            position, speed, result, error = self.sdk.ReadPosSpeed(servo_id)
            if result != COMM_SUCCESS or error:
                raise ServoError(
                    f"ID {servo_id} read position: {self.sdk.getTxRxResult(result)} "
                    f"(servo error {error})"
                )
            return position, speed

    def read_limits(self, servo_id: int) -> tuple[int, int]:
        low = self.read_register(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L, 2)
        high = self.read_register(servo_id, SMS_STS_MAX_ANGLE_LIMIT_L, 2)
        if not 0 <= low < high <= 4095:
            raise ServoError(f"ID {servo_id} invalid position limits [{low}, {high}]")
        return low, high

    def read_torque(self, servo_id: int) -> int:
        return self.read_register(servo_id, SMS_STS_TORQUE_ENABLE)

    def arm(self, servo_id: int) -> None:
        """Set the current position as goal before enabling position holding."""
        with self.lock:
            if self.read_register(servo_id, SMS_STS_MODE) != 0:
                raise ServoError(f"ID {servo_id} is not in position mode")
            torque = self.read_torque(servo_id)
            if torque == 1:
                return
            if torque != 0:
                raise ServoError(f"ID {servo_id} has unexpected torque state {torque}")
            position, _ = self.read_position_speed(servo_id)
            low, high = self.read_limits(servo_id)
            if not low <= position <= high:
                raise ServoError(f"ID {servo_id} is outside its configured limits")
            self._written(
                self.sdk.WritePosEx(servo_id, position, 10, 5),
                "set current goal before torque enable",
                servo_id,
            )
            self._written(
                self.sdk.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 1),
                "enable torque",
                servo_id,
            )

    def release(self, servo_id: int) -> None:
        with self.lock:
            self._written(
                self.sdk.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 0),
                "disable torque",
                servo_id,
            )

    def command_position(
        self, servo_id: int, position: int, speed: int = 20, acceleration: int = 10
    ) -> None:
        self._check_id(servo_id)
        if type(position) is not int or not 0 <= position <= 4095:
            raise ValueError("position must be an integer encoder count from 0 to 4095")
        if type(speed) is not int or not 1 <= speed <= 1000:
            raise ValueError("speed must be an integer from 1 to 1000")
        if type(acceleration) is not int or not 1 <= acceleration <= 100:
            raise ValueError("acceleration must be an integer from 1 to 100")
        with self.lock:
            low, high = self.read_limits(servo_id)
            if not low <= position <= high:
                raise ServoError(
                    f"ID {servo_id} target {position} outside device limits [{low}, {high}]"
                )
            if self.read_register(servo_id, SMS_STS_MODE) != 0:
                raise ServoError(f"ID {servo_id} is not in position mode")
            if self.read_torque(servo_id) != 1:
                raise ServoError(f"ID {servo_id} torque is off; arm explicitly first")
            self._written(
                self.sdk.WritePosEx(servo_id, position, speed, acceleration),
                "write position",
                servo_id,
            )

    def sync_command_positions(
        self, goals: dict[int, int], speed: int = 10, acceleration: int = 5
    ) -> int:
        """Legacy uniform-profile interface; one broadcast, as before."""
        return self.sync_command_positions_profiled(
            goals, {i: {'speed_raw': speed, 'acceleration_raw': acceleration} for i in goals})

    def sync_command_positions_profiled(self, goals, profiles, *, require_torque=True):
        """One broadcast carrying each axis's final goal AND individual profile.

        require_torque=False is ONLY for fresh, stopped, current-position holding.
        It does not permit sending a movement goal to an unpowered arm.
        Broadcast writes have no acknowledgement; caller must verify feedback.
        """
        if not isinstance(goals, dict) or not 2 <= len(goals) <= 6:
            raise ValueError('sync command requires 2 to 6 IDs')
        if not isinstance(profiles, dict) or set(profiles) != set(goals):
            raise ValueError('profile IDs must equal goal IDs')
        for servo_id, position in goals.items():
            self._check_id(servo_id)
            if not 1 <= servo_id <= 6:
                raise ValueError('project sync supports IDs 1–6')
            if type(position) is not int or not 0 <= position <= 4095:
                raise ValueError(f'ID{servo_id}: invalid goal')
            profile = profiles[servo_id]
            speed, acc = profile['speed_raw'], profile['acceleration_raw']
            if type(speed) is not int or not 1 <= speed <= 1000:
                raise ValueError(f'ID{servo_id}: invalid speed')
            if type(acc) is not int or not 1 <= acc <= 100:
                raise ValueError(f'ID{servo_id}: invalid acceleration')
        with self.lock:
            for servo_id, position in goals.items():
                low, high = self.read_limits(servo_id)
                if not low <= position <= high:
                    raise ServoError(f'ID{servo_id}: target outside device limits')
                if self.read_register(servo_id, SMS_STS_MODE) != 0:
                    raise ServoError(f'ID{servo_id}: not in position mode')
                torque = self.read_torque(servo_id)
                if torque not in (0, 1) or (require_torque and torque != 1):
                    raise ServoError(f'ID{servo_id}: unexpected torque state')
                if not require_torque:
                    current, velocity = self.read_position_speed(servo_id)
                    if abs(position - current) > 2 or velocity != 0:
                        raise ServoError(f'ID{servo_id}: hold initialization requires fresh stopped position')
            group = self.sdk.groupSyncWrite
            group.clearParam()
            try:
                for servo_id in sorted(goals):
                    profile = profiles[servo_id]
                    if not self.sdk.SyncWritePosEx(servo_id, goals[servo_id],
                                                 profile['speed_raw'], profile['acceleration_raw']):
                        raise ServoError(f'ID{servo_id}: sync packet assembly failed')
                sent_at_ns = time.monotonic_ns()
                result = group.txPacket()
                if result != COMM_SUCCESS:
                    raise ServoError(f'sync position write: {self.sdk.getTxRxResult(result)}')
                return sent_at_ns
            finally:
                group.clearParam()

    def read_control_feedback(self, servo_id):
        """Fresh contiguous registers 40–70; one read request per servo.

        Byte widths and sign-magnitude speed decoding match the STS SDK.
        Raises on any packet/status error; never returns stale cached values.
        """
        self._check_id(servo_id)
        with self.lock:
            data, result, error = self.sdk.readTxRx(servo_id, 40, 31)
            if result != COMM_SUCCESS or error or len(data) != 31:
                raise ServoError(f'ID{servo_id} feedback: {self.sdk.getTxRxResult(result)} (error={error}, length={len(data)})')
            def word(address):
                k = address - 40
                return data[k] | (data[k+1] << 8)
            return {'id': servo_id, 'torque': data[0], 'acceleration': data[1],
                    'goal': word(42), 'command_speed_raw': word(46),
                    'position': self.sdk.scs_tohost(word(56), 15),
                    'speed': self.sdk.scs_tohost(word(58), 15),
                    'load_raw': word(60), 'voltage_decivolts': data[22],
                    'temperature_c': data[23], 'moving': data[26],
                    'current_raw': word(69)}

    def sample(self, servo_id: int) -> dict[str, int]:
        """Read feedback for diagnosing position holding; no register writes."""
        with self.lock:
            monotonic_ns = time.monotonic_ns()
            wall_ns = time.time_ns()
            position, speed = self.read_position_speed(servo_id)
            return {
                "monotonic_ns": monotonic_ns,
                "wall_ns": wall_ns,
                "id": servo_id,
                "goal": self.read_register(servo_id, SMS_STS_GOAL_POSITION_L, 2),
                "position": position,
                "speed": speed,
                "load_raw": self.read_register(servo_id, SMS_STS_PRESENT_LOAD_L, 2),
                "current_raw": self.read_register(servo_id, SMS_STS_PRESENT_CURRENT_L, 2),
                "voltage_decivolts": self.read_register(servo_id, SMS_STS_PRESENT_VOLTAGE),
                "temperature_c": self.read_register(servo_id, SMS_STS_PRESENT_TEMPERATURE),
                "torque": self.read_torque(servo_id),
            }
