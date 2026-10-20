#!/usr/bin/env python3
"""Check firmware interfaces/assets for the requested wake word; no motion commands."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
from zoneinfo import ZoneInfo
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
from host.transport.download_calibration import CameraLink, PORT

BOARD_PROBE = '''import os,sys,json
report={'version':sys.version,'interfaces':{},'assets':{}}
checks=[('uart','machine','UART'),('fpioa','machine','FPIOA'),
        ('camera','media.sensor','Sensor'),('audio','media.pyaudio','PyAudio'),
        ('kpu','nncase_runtime','kpu'),('wake_word','open_kws','Engine')]
for key,module,attribute in checks:
 try:
  imported=__import__(module,None,None,[attribute])
  report['interfaces'][key]=hasattr(imported,attribute)
 except ImportError as exc:
  report['interfaces'][key]=False
  report[key+'_error']=str(exc)
try:
 from machine import FPIOA
 report['uart_pins']=all(hasattr(FPIOA,n) for n in ['UART1_TXD','UART1_RXD','UART2_TXD','UART2_RXD'])
except ImportError:report['uart_pins']=False
root='/sdcard/examples/31-OPEN-KWS/'
for name in ['model/kws.kmodel','assets','units.txt','open_kws_app.py']:
 try:
  st=os.stat(root+name)
  report['assets'][name]={'present':True,'directory':bool(st[0]&0x4000),'bytes':st[6]}
  if name=='assets':report['assets'][name]['files']=os.listdir(root+name)
 except OSError:report['assets'][name]={'present':False}
print('VOICE_CAPABILITIES '+json.dumps(report))
print('EXPORT_DONE')
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default=PORT)
    args=parser.parse_args()
    requested=json.loads((ROOT/'configs/voice/wake_word.json').read_text())
    folder=ROOT/'data/logs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('k230_voice_capabilities_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    record={'requested':requested,'motion_commands':False,'status':'failed'}
    link=None
    try:
        link=CameraLink(args.port)
        report=None
        for raw in link.execute(BOARD_PROBE,timeout=20):
            line=raw.decode(errors='replace')
            if line.startswith('VOICE_CAPABILITIES '):report=json.loads(line[19:])
            else:print(line,flush=True)
        if report is None:raise RuntimeError('No capability report returned')
        record['capabilities']=report
        available=(all(report['interfaces'].values()) and report['uart_pins'] and
                   all(a['present'] for a in report['assets'].values()) and
                   bool(report['assets'].get('assets',{}).get('files')))
        record['status']='interfaces_and_paths_present' if available else 'upgrade_or_assets_required'
        print(json.dumps(record,ensure_ascii=False,indent=2),flush=True)
        # Presence alone does not verify model loading, camera capture or recognition accuracy.
        return 0 if available else 2
    except Exception as error:
        record['error']=str(error)
        print('VOICE CHECK FAILED:',error,flush=True)
        return 1
    finally:
        if link is not None:link.port.close()
        (folder/'record.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
        print('RECORD',folder,flush=True)


if __name__=='__main__':raise SystemExit(main())
