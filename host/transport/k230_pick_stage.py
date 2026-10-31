"""PC-owned stage dispatch over K230 USBDBG -> existing UART1 arm helpers."""
import ast
import json
from host.project_paths import ROOT
from host.arm.task_controller import REGISTERS
from host.transport.download_calibration import CameraLink,PORT


def payload(config,goals):
    config.check_counts(dict(enumerate(goals,1)))
    return {'goals':goals,'profiles':[config.profiles[i] for i in range(1,7)],
        'registers':[{str(a):config.baseline['parameters'][str(i)][name]
                      for a,name in REGISTERS.items()} for i in range(1,7)],
        'software_limits':[config.mapping.joint(i)['software_count_limits'] for i in range(1,7)]}


def build_action(config,action,stage=None):
    read=(ROOT/'device/k230/diagnostics/uart1_servo_read.py').read_text().split('\nif __name__ == "__main__":')[0]
    ready=(ROOT/'device/k230/arm/ready_pose.py').read_text()
    # Reuse precisely the same pure position-stillness rule as the V0 flow.
    source=(ROOT/'host/arm/blue_pick_settle.py').read_text()
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='PositionSettled')
    settle='\n'.join(source.splitlines()[cls.lineno-1:cls.end_lineno])
    runner=(ROOT/'device/k230/arm/pick_stage.py').read_text()
    source='\n'.join((read,ready,settle,runner))
    if action=='stage_direct':
        # Execute the exact existing standalone stage loop, without its legacy
        # fixed-duration preflight/dispatch branch or another implementation.
        source+='\nrun_pick_stage_direct(ujson.loads(%r),ujson.loads(%r))\n' % (json.dumps(config),json.dumps(stage))
        source+='print("PICK_ACTION_PASS stage_direct")\n'
    else:
        source+='\nrun_pick_action(ujson.loads(%r),%r,ujson.loads(%r))\n' % (json.dumps(config),action,json.dumps(stage))
    return 'try:\n exec('+repr(source)+')\nexcept BaseException as exc:\n import sys\n sys.print_exception(exc)\nfinally:\n print("EXPORT_DONE")\n'


def run_action(config,action,stage=None,port=PORT,emit=lambda event:None):
    link=None
    events=[]
    passed=False
    try:
        link=CameraLink(port)
        if action=='stage_direct' and stage['name'] in ('hover','descend','close'):
            # Position/sample gates have no elapsed-time advance or host deadline.
            timeout=float('inf')
        else:
            timeout=86400 if stage and stage['name']=='close' else 45
        for raw in link.execute(build_action(config,action,stage),timeout=timeout):
            text=raw.decode(errors='replace')
            if text.startswith(('PICK_EVENT ','ARM_EVENT ')):
                event=json.loads(text.split(' ',1)[1]);events.append(event);emit(event)
            elif text=='PICK_ACTION_PASS '+action:passed=True
            elif text.strip():emit({'event':'board_message','text':text})
        if not passed:raise RuntimeError('K230 action failed; no next stage dispatched')
        return events
    except BaseException:
        if link is not None:
            # Stop only the current board script, never torque-off or recovery.
            try:link.command(0x06)
            except Exception:pass
        raise
    finally:
        if link is not None:link.port.close()
