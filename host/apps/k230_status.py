#!/usr/bin/env python3
"""Passive K230 USB CDC status monitor: receive only, never transmit bytes."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT
import argparse
import codecs
from datetime import datetime
import json
import math
import time
from zoneinfo import ZoneInfo
import serial

PORT = '/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00'

STAGES={'hover':'上方悬停','descend':'下探','close':'闭合夹爪','lift':'抬升'}


def open_receiver(port):
    if port == PORT and not Path(port).exists():
        devices = sorted(Path('/dev/serial/by-id').glob('usb-Kendryte_CanMV_*-if00'))
        if len(devices) > 1:
            raise RuntimeError('找到多块CanMV，请用--port指定设备')
        if len(devices) == 1:port = str(devices[0])
    # No USBDBG header, REPL input, reset or serial write. The firmware switches
    # raw stdout on with CDC DTR; a USBDBG request would attach the IDE and can
    # interrupt boot main.py. Never replace this receiver with CameraLink.
    return serial.Serial(port,921600,timeout=.1,exclusive=True)


class StatusDisplay:
    def __init__(self,emit,feedback_period=1.,raw=False,clock=time.monotonic):
        self.emit=emit;self.feedback_period=feedback_period;self.raw=raw;self.clock=clock
        self.decoder=codecs.getincrementaldecoder('utf-8')(errors='replace')
        self.pending='';self.last_event=None;self.last_event_at=None;self.last_feedback=-math.inf
    def feed(self,data):
        self.pending+=self.decoder.decode(data)
        while '\n' in self.pending:
            line,self.pending=self.pending.split('\n',1)
            self.line(line.rstrip('\r').replace('\x00',''))
        if len(self.pending)>131072:
            self.emit('非标准长日志：'+self.pending[:500]);self.pending=''
    def line(self,line):
        if self.raw and line.strip():self.emit('原始：'+line)
        for prefix in ('AIC_PICK ','PICK_EVENT ','ARM_EVENT '):
            index=line.find(prefix)
            if index<0:continue
            try:
                record=json.loads(line[index+len(prefix):])
                if not isinstance(record,dict):raise ValueError('event is not an object')
                event=record.get('event','unknown')
                details=record.get('details')
                if event=='arm_feedback':
                    if not isinstance(details,dict):raise ValueError('invalid arm feedback')
                    self.arm(details)
                elif prefix!='AIC_PICK ':self.arm(record)
                else:self.application(event,details)
            except (ValueError,TypeError,KeyError) as exc:
                self.emit('状态日志解析失败：'+str(exc)+'；'+line[:300])
            return
        if line.strip() and not self.raw:self.emit('板端：'+line)
    def state(self,message):
        self.last_event=message;self.last_event_at=self.clock();self.emit(message)
    def application(self,event,details):
        messages={'startup_waiting_for_wake':'程序启动，正在初始化语音',
                  'voice_ready':'监听就绪：现在可以说“思达思达”',
                  'wake':'已识别唤醒词：正在准备拍照识别',
                  'planned':'四阶段规划完成，准备执行',
                  'torque_on':'机械臂扭矩保持已开启',
                  'complete_torque_on':'本轮流程完成，保持抬升姿态和扭矩；实际夹取结果需观察'}
        if event=='localized':
            center=details.get('cube_center_Base_mm') if isinstance(details,dict) else None
            text='识别成功'
            if isinstance(center,list) and len(center)==3:
                text+='：方块Base坐标(mm) ['+', '.join(f'{v:.2f}' for v in center)+']'
            self.state(text)
        elif event=='pick_failed_no_extra_motion':
            self.state('本轮失败，未继续后续动作：'+str(details))
        elif event in messages:self.state(messages[event])
        else:self.state('事件 '+str(event)+'：'+json.dumps(details,ensure_ascii=False))
    def arm(self,event):
        name=STAGES.get(event.get('name'),event.get('name',''))
        kind=event.get('event')
        if kind=='stage_started':self.state('执行阶段：'+name)
        elif kind=='stage_complete':self.state('阶段完成：'+name)
        elif kind=='gripper_settled':self.state('夹爪已满足连续0.5秒静止条件，准备抬升')
        elif kind=='stage_feedback':
            now=self.clock()
            if self.feedback_period<=0 or now-self.last_feedback<self.feedback_period:return
            self.last_feedback=now;rows=event.get('rows',[])
            positions=' '.join(str(r.get('position','?')) for r in rows)
            errors=[r['position']-r['goal'] for r in rows if isinstance(r.get('position'),(int,float)) and isinstance(r.get('goal'),(int,float))]
            text=f'[{name}] 六轴编码：{positions}'
            if errors:text+='；位置−目标：'+' '.join(f'{v:+g}' for v in errors)
            self.emit(text)
        elif kind in ('holding_current','preflight_verified','sync_goal_verified'):return
        else:self.emit('机械臂事件：'+json.dumps(event,ensure_ascii=False))
    def heartbeat(self):
        if self.last_event_at is None:
            self.emit('串口已打开，尚未收到应用状态；静默不能说明程序是否运行')
        else:
            self.emit(f'串口已打开；最近事件：{self.last_event}（{self.clock()-self.last_event_at:.0f}秒前）')


def monitor(port,seconds=None,raw=False,feedback_period=1.,output=None,
            receiver_factory=open_receiver,clock=time.monotonic,sleep=time.sleep):
    folder=Path(output) if output else ROOT/'data/logs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('k230_status_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=True)
    receiver=None;started=clock();last_error=None
    with (folder/'board.log').open('ab') as board,(folder/'status.log').open('a',buffering=1) as status:
        def emit(message):
            line=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('[%H:%M:%S] ')+message
            print(line,flush=True);status.write(line+'\n')
        emit('纯接收监视：不向K230发送任何字节，只显示连接后收到的日志；Ctrl+C仅关闭本机监视')
        try:
            while seconds is None or clock()-started<seconds:
                try:
                    if receiver is None:
                        receiver=receiver_factory(port);display=StatusDisplay(emit,feedback_period,raw,clock)
                        emit('串口已打开：'+str(getattr(receiver,'port',port)))
                        next_heartbeat=clock()+10;last_error=None
                    data=receiver.read(min(max(receiver.in_waiting,1),65536))
                    if data:board.write(data);board.flush();display.feed(data)
                    now=clock()
                    if now>=next_heartbeat:
                        display.heartbeat();next_heartbeat=now+10
                    sleep(.1)
                except (OSError,TimeoutError,RuntimeError) as exc:
                    if receiver is not None:
                        try:receiver.close()
                        except Exception:pass
                    receiver=None;message=str(exc)
                    if message!=last_error:
                        emit('监视连接不可用：'+message+'；将重试，K230是否继续运行不能据此判断')
                        last_error=message
                    sleep(1)
        except KeyboardInterrupt:pass
        finally:
            if receiver is not None:receiver.close()
            emit('监视结束；全程未向K230发送数据，未启动或停止板端程序')
            print('LOG',folder,flush=True)
    return folder


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default=PORT)
    p.add_argument('--seconds',type=float,help='optional duration; otherwise keep monitoring')
    p.add_argument('--raw',action='store_true',help='also display every raw board line')
    p.add_argument('--feedback-period',type=float,default=1.,help='seconds between joint feedback lines; 0 hides them')
    p.add_argument('--output',type=Path)
    a=p.parse_args(argv)
    if a.seconds is not None and (not math.isfinite(a.seconds) or a.seconds<=0):p.error('seconds must be positive and finite')
    if not math.isfinite(a.feedback_period) or a.feedback_period<0:p.error('feedback-period must be nonnegative and finite')
    monitor(a.port,a.seconds,a.raw,a.feedback_period,a.output)
    return 0

if __name__=='__main__':raise SystemExit(main())
