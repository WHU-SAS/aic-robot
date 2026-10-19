#!/usr/bin/env python3
"""Read-only Linux probe for the ST-3215-C018 arm servos.

Uses the bundled Feetech SMS/STS SDK. It sends only ping and read requests;
it never enables torque or writes a target position.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)

import argparse
import glob
import sys
from pathlib import Path

SDK_ROOT = REPO_ROOT / "third_party" / "ftservo_python"

from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts  # noqa: E402


def choose_port(explicit_port):
    if explicit_port:
        return explicit_port
    candidates = sorted(glob.glob("/dev/serial/by-id/*"))
    candidates += sorted(glob.glob("/dev/ttyACM*"))
    candidates += sorted(glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise RuntimeError("未找到串口设备；请检查 USB 连接，或用 --port 指定设备")
    return candidates[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00", help="机械臂专用串口；不自动选择相机串口")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="+", default=list(range(1, 7)))
    args = parser.parse_args()

    if not args.ids or any(servo_id < 0 or servo_id > 253 for servo_id in args.ids):
        parser.error("舵机 ID 必须在 0 到 253 之间")

    try:
        port_name = choose_port(args.port)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

    port = PortHandler(port_name)
    try:
        port.baudrate = args.baud
        if not port.openPort():
            print(f"无法打开串口或设置波特率：{port_name} @ {args.baud}", file=sys.stderr)
            return 2
        servo = sms_sts(port)
        print(f"串口：{port_name}，波特率：{args.baud}；仅发送 ping 和读取请求")
        found = 0
        for servo_id in args.ids:
            model, result, error = servo.ping(servo_id)
            if result != COMM_SUCCESS or error:
                print(f"ID {servo_id}: 无响应或报错 ({servo.getTxRxResult(result)}, error={error})")
                continue
            position, speed, result, error = servo.ReadPosSpeed(servo_id)
            if result != COMM_SUCCESS or error:
                print(f"ID {servo_id}: 型号代码 {model}；读取位置失败 "
                      f"({servo.getTxRxResult(result)}, error={error})")
                continue
            found += 1
            print(f"ID {servo_id}: 型号代码 {model}，位置 {position}，速度 {speed}")
        print(f"读取成功：{found}/{len(args.ids)} 个舵机")
        return 0 if found == len(args.ids) else 1
    except (OSError, ValueError) as exc:
        print(f"串口访问失败：{exc}", file=sys.stderr)
        return 2
    finally:
        if port.is_open:
            port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
