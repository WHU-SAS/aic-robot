#!/usr/bin/env python3
"""Test the K230 onboard microphone, optionally save a bounded recording."""
import argparse
import base64
from datetime import datetime
import json
from pathlib import Path
import sys
from zoneinfo import ZoneInfo
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
from host.transport.download_calibration import CameraLink, PORT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--seconds', type=int, choices=range(1, 16), default=3)
    parser.add_argument('--record', action='store_true', help='Save a WAV to SD and local ignored logs')
    args = parser.parse_args()
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    folder = ROOT / 'data/logs' / ('k230_audio_' + stamp)
    folder.mkdir(parents=True)
    remote = '/sdcard/voice_probe/mic_' + stamp + '.wav' if args.record else None
    source = (ROOT / 'device/k230/audio/microphone_probe.py').read_text()
    source += '\nrun_microphone_probe(' + repr(args.seconds) + ',' + repr(remote) + ')\n'
    script = 'try:\n exec(' + repr(source) + ')\nexcept BaseException as e:\n import sys\n sys.print_exception(e)\nfinally:\n print("EXPORT_DONE")\n'
    link = None
    lines = []
    result = {'status': 'failed', 'motion_commands': False, 'recording_requested': args.record}
    try:
        link = CameraLink(args.port)
        for raw in link.execute(script, timeout=args.seconds + 25):
            line = raw.decode(errors='replace')
            lines.append(line)
            print(line, flush=True)
            if line.startswith('MIC_RESULT '):
                result['microphone'] = json.loads(line[11:])
        if 'MIC_PROBE_PASS' not in lines:
            raise RuntimeError('Microphone check did not complete')
        if remote:
            data = bytearray()
            fetch = ('import binascii,time\nwith open(' + repr(remote) + ',"rb") as f:\n'
                     ' while True:\n  b=f.read(768)\n  if not b: break\n'
                     '  print("AUDIO_DATA "+binascii.b2a_base64(b).decode().strip())\n'
                     '  time.sleep_ms(5)\nprint("EXPORT_DONE")\n')
            for raw in link.execute(fetch, timeout=45):
                if raw.startswith(b'AUDIO_DATA '):
                    data.extend(base64.b64decode(raw[11:], validate=True))
            expected = 44 + args.seconds * 16000 * 2
            if len(data) != expected or data[:4] != b'RIFF' or data[8:12] != b'WAVE':
                raise RuntimeError('Incomplete WAV transfer')
            target = folder / 'microphone.wav'
            target.write_bytes(data)
            result['wav'] = str(target)
        result['status'] = 'passed'
        return 0
    except Exception as error:
        result['error'] = str(error)
        print('AUDIO FAILED:', error, flush=True)
        return 1
    finally:
        if link is not None:
            link.port.close()
        (folder / 'record.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        (folder / 'board_log.txt').write_text('\n'.join(lines) + '\n')
        print('RECORD', folder, flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
