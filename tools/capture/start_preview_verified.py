#!/usr/bin/env python3
"""Start preview and verify USB framebuffer output; does not save photographs."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
from pathlib import Path
import io
import struct
import time
from PIL import Image
from download_calibration import CameraLink, PORT

link=CameraLink(PORT)
try:
    link.command(0x80,12)
    print('FIRMWARE',struct.unpack('<III',link.read(12)),flush=True)
    link.command(0x06)
    time.sleep(.15)
    link.output()
    link.command(0x0D,4)
    link.port.write(struct.pack('<I',1));link.port.flush()
    time.sleep(.05)
    payload=(REPO_ROOT/'device/k230/diagnostics/camera_preview.py').read_bytes()
    if len(payload)%64==0:payload+=b'\n'
    link.command(0x05,len(payload));time.sleep(.06)
    link.port.write(payload);link.port.flush()
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        log=link.output()
        if log:print(log.decode(errors='replace'),end='',flush=True)
        link.command(0x81,12)
        width,height,bpp=struct.unpack('<III',link.read(12))
        if width and height and 3<bpp<10_000_000:
            link.command(0x82,bpp)
            jpeg=link.read(bpp)
            with Image.open(io.BytesIO(jpeg)) as frame:
                frame.load()
                if frame.size!=(640,480):raise RuntimeError('Unexpected preview dimensions')
            link.command(0x87,4)
            running=struct.unpack('<I',link.read(4))[0]
            print('\nPREVIEW_VERIFIED %dx%d JPEG=%d script_running=%d'%(width,height,bpp,running),flush=True)
            break
        time.sleep(.1)
    else:
        raise RuntimeError('No valid framebuffer image within 15 seconds')
finally:
    link.port.close()
