#!/usr/bin/env python3
"""Run the existing K230 UART2/MSPM0 UART1 test over USBDBG; no servo commands."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
from host.transport.download_calibration import CameraLink, PORT
import argparse
from datetime import datetime
import json
import re
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT, help='K230 USBDBG serial port; stop and disconnect CanMV IDE first')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    out = args.output or ROOT / 'data/logs' / datetime.now().strftime('k230_mspm0_link_%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'device/k230/diagnostics/k230_uart2_test.py').read_text()
    script = ('try:\n exec(' + repr(source) + ')\n'
              'except BaseException as exc:\n import sys\n sys.print_exception(exc)\n'
              'finally:\n print("EXPORT_DONE")\n')
    link = None
    lines = []
    result = {'status': 'failed', 'port': args.port, 'rounds': {}, 'firmware_flashed': False,
              'servo_commands_sent': False}
    started = time.monotonic()
    try:
        link = CameraLink(args.port)
        link.command(0x80, 12)
        result['k230_firmware'] = list(struct.unpack('<III', link.read(12)))
        for raw in link.execute(script, timeout=30):
            line = raw.decode(errors='replace')
            lines.append(line)
            print(line, flush=True)
            match = re.match(r'^Round (\d+) (PASS|FAIL) ', line)
            if match:
                seq = int(match.group(1))
                if str(seq) in result['rounds']:
                    raise RuntimeError('Duplicate round number from K230')
                result['rounds'][str(seq)] = match.group(2)
        expected = {str(n): 'PASS' for n in range(1, 21)}
        if result['rounds'] != expected or 'RESULT: 20 / 20' not in lines or 'UART2_MSPM0_LINK_PASS' not in lines:
            raise RuntimeError('UART link did not pass all 20 request/ACK rounds; inspect board_log.txt')
        result['status'] = 'passed'
        return 0
    except Exception as exc:
        result['error'] = str(exc)
        print('UART TEST FAILED:', exc, file=sys.stderr, flush=True)
        return 1
    finally:
        if link is not None:
            link.port.close()
        result['elapsed_s'] = time.monotonic() - started
        (out / 'board_log.txt').write_text('\n'.join(lines) + '\n')
        (out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('RECORD', out, flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
