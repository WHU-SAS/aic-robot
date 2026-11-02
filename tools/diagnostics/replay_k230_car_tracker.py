#!/usr/bin/env python3
"""Replay saved K230 JPEGs through native blobs, without camera/UART motion.

Default builds a script only. --execute runs saved-file processing; optional
--restore-listener resumes the existing SD startup without installing a release.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import glob
import json
import struct
import subprocess
import time
from host.project_paths import ROOT
from host.car.runtime import load
from host.transport.download_calibration import CameraLink, PORT


def build_script(config, geometry, paths):
    sources = [(ROOT / p).read_text() for p in (
        'device/k230/car/ground_projection.py', 'device/k230/car/blue_tracker.py')]
    # This transient script execs code in __main__, outside a Python package.
    # CanMV raises KeyError (not ImportError) for the relative import there.
    # Use the existing release's flat pose module before executing the class.
    sources[0] = sources[0].replace('from ..pick.pose import undistort, project',
                                    'from pose import undistort, project', 1)
    model = {'K': geometry.K, 'D': geometry.D, 'T': geometry.T,
             'plane': {'normal': geometry.normal, 'offset_mm': geometry.offset}}
    return '''import sys,json,time,gc,image
try:
 with open('/sdcard/aic/pick/current.json') as f:release=json.load(f)['release']
 sys.path.insert(0,release)
 for source in %r:exec(source,globals())
 model=%r
 projection=GroundProjection(model['K'],model['D'],model['T'],model['plane'])
 for path in %r:
  img=image.Image(path).to_rgb565()
  tracker=BlueTracker(%r,projection)
  durations=[]
  for unused in range(3):
   started=time.ticks_ms()
   observation=tracker.detect_frame(img,started)
   durations.append(time.ticks_diff(time.ticks_ms(),started))
  print('CAR_REPLAY '+json.dumps({'path':path,'frame_ms':durations,'observation':observation,'repeated_saved_still':True}))
  img=None;gc.collect()
except BaseException as exc:
 sys.print_exception(exc)
finally:print('EXPORT_DONE')
''' % (sources, model, paths, config['tracking'])


def restore_listener(link):
    """Hard-boot the saved main, then observe it without attaching USBDBG.

    The old link is closed here. Keeping main inside an IDE-uploaded script
    would stop it again when that link's DTR drops; USBDBG 0x0c is only a soft
    reset on this board and does not restore boot auto-execution.
    """
    # Delayed import: the motor diagnostic also uses this recovery helper.
    from tools.diagnostics.test_k230_motor_uart import stop_previous_script
    from host.apps.k230_status import open_receiver
    port = link.port.port
    source = b"import machine\nprint('AIC_SOC_RESET_REQUESTED')\nmachine.reset()\n"
    if len(source) % 64 == 0:
        source += b'\n'  # Keep the existing CanMV USB script-transfer workaround.
    try:
        stop_previous_script(link)
        link.command(0x05, len(source))
        time.sleep(.06)
        if link.port.write(source) != len(source):
            raise RuntimeError('Incomplete K230 hard-reset script upload')
        try:
            link.port.flush()
        except OSError:
            pass  # Reset may disconnect USB immediately after receiving it.
        # Let reset execute before a host DTR drop could stop the uploaded code.
        time.sleep(1)
    finally:
        link.port.close()
    lines = []; pending = b''; receiver = None; started = time.monotonic()
    failure = False; last_output = started
    try:
        while time.monotonic() - started < 50:
            try:
                if receiver is None:
                    receiver = open_receiver(port)
                chunk = receiver.read(min(max(receiver.in_waiting, 1), 65536))
            except (OSError, TimeoutError):
                if receiver is not None:
                    receiver.close()
                    receiver = None
                time.sleep(.1)
                continue  # SoC reset removes and re-enumerates the USB device.
            if chunk: last_output = time.monotonic()
            pending += chunk
            while b'\n' in pending:
                raw, pending = pending.split(b'\n', 1)
                line = raw.decode(errors='replace').strip().replace('\x00', '')
                lines.append(line)
                if 'Traceback' in line or 'voice_failed' in line:
                    failure = True
                if not failure and ('"event": "voice_ready"' in line or '"event":"voice_ready"' in line):
                    return lines
            if failure and time.monotonic() - last_output >= .5:
                if pending: lines.append(pending.decode(errors='replace'))
                raise RuntimeError('Listener failed:\n' + '\n'.join(lines))
            time.sleep(.05)
        if pending: lines.append(pending.decode(errors='replace'))
        raise TimeoutError('Listener did not report voice_ready after hard reset:\n' + '\n'.join(lines))
    finally:
        if receiver is not None:
            receiver.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--remote-image', action='append', default=[])
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--restore-listener', action='store_true')
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not args.remote_image: parser.error('Provide saved SD image paths; no camera capture is performed')
    if args.restore_listener and not args.execute: parser.error('--restore-listener requires --execute')
    for path in args.remote_image:
        if not path.startswith('/sdcard/') or '..' in path.split('/'):
            parser.error('Expected a saved /sdcard/ image path')
    unused, config, geometry = load()
    output = args.output or ROOT / 'data/logs' / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('car_tracker_board_%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=True)
    script = build_script(config, geometry, args.remote_image)
    (output / 'board_script.py').write_text(script)
    result = {'status': 'built_no_hardware_access', 'new_captures': 0, 'motion_performed': False,
              'replays': [], 'listener_restored': False, 'repeated_stills_not_live_tracking': True}
    link = None; lines = []
    try:
        if args.execute:
            port = args.port
            if port == PORT and not Path(port).exists():
                ports = glob.glob('/dev/serial/by-id/usb-Kendryte_CanMV_*-if00')
                if len(ports) != 1: raise RuntimeError('Expected one K230')
                port = ports[0]
            owner = subprocess.run(['fuser', port], text=True, capture_output=True)
            if owner.returncode != 1:
                raise RuntimeError('Port busy or ownership check failed: ' + owner.stdout + owner.stderr)
            link = CameraLink(port); link.command(0x80, 12)
            result['firmware'] = list(struct.unpack('<III', link.read(12)))
            for raw in link.execute(script, timeout=90):
                line = raw.decode(errors='replace'); lines.append(line)
                if line.startswith('CAR_REPLAY '):
                    row = json.loads(line[len('CAR_REPLAY '):]); result['replays'].append(row)
                    print(json.dumps({'path': row['path'], 'frame_ms': row['frame_ms'],
                        'status': row['observation']['status'], 'bbox_px': row['observation'].get('bbox_px')}, ensure_ascii=False), flush=True)
            if len(result['replays']) != len(args.remote_image):
                raise RuntimeError('Not every saved image replay completed; see board_log.txt')
            result['status'] = 'native_saved_image_replay_completed'
    except Exception as exc:
        result.update(status='failed', error=repr(exc)); print(str(exc), file=sys.stderr)
    finally:
        if link:
            if args.restore_listener:
                try:
                    restore_lines = restore_listener(link)
                    (output / 'listener_log.txt').write_text('\n'.join(restore_lines) + '\n')
                    result['listener_restored'] = True
                except Exception as exc:
                    result['listener_restore_error'] = repr(exc)
                    result['status'] = 'listener_restore_failed'
                    print('LISTENER_RESTORE_FAILED', repr(exc), file=sys.stderr)
            link.port.close()
        (output / 'board_log.txt').write_text('\n'.join(lines) + '\n')
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('RECORD', output, flush=True)
    return 0 if result['status'] in ('built_no_hardware_access', 'native_saved_image_replay_completed') else 1


if __name__ == '__main__': raise SystemExit(main())
