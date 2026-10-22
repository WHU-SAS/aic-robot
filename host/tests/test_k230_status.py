"""Passive serial receiver and status translation; no hardware."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from host.apps.k230_status import StatusDisplay,monitor,open_receiver

class Clock:
    def __init__(self):self.now=0.
    def __call__(self):return self.now
    def sleep(self,s):self.now+=s

class DisplayTests(unittest.TestCase):
    def setUp(self):
        self.lines=[];self.clock=Clock();self.display=StatusDisplay(self.lines.append,clock=self.clock)
    def feed(self,event,details=None):
        self.display.feed(('AIC_PICK '+json.dumps({'event':event,'details':details},ensure_ascii=False)+'\n').encode())
    def test_split_utf8_and_lines_preserve_chinese_wake_event(self):
        data=('AIC_PICK '+json.dumps({'event':'voice_ready','details':{'word':'思达思达'}},ensure_ascii=False)+'\r\n').encode()
        for b in data:self.display.feed(bytes([b]))
        self.assertEqual(len(self.lines),1)
        self.assertIn('监听就绪',self.lines[0]);self.assertNotIn('\ufffd',self.lines[0])
        self.feed('wake',{'label':'sida'});self.assertIn('已识别唤醒词',self.lines[-1])
    def test_ready_voice_status_does_not_announce_pick_or_confirm_arrival(self):
        self.feed('voice_ready',{'words':['思达思达','回准备位']})
        self.assertIn('回准备位',self.lines[-1]);self.assertIn('思达思达',self.lines[-1])
        self.feed('wake',{'word':'回准备位','action':'ready'})
        self.assertIn('准备位动作',self.lines[-1]);self.assertNotIn('拍照',self.lines[-1])
        self.feed('ready_dispatched')
        self.assertIn('未复读或等待到位',self.lines[-1])
        self.feed('ready_failed_no_extra_motion','hold failed')
        self.assertIn('hold failed',self.lines[-1])

    def test_nested_arm_events_and_feedback_throttle(self):
        self.feed('arm_feedback',{'event':'stage_started','name':'descend'})
        self.assertIn('下探',self.lines[-1])
        row={'id':1,'position':2050,'goal':2048}
        event={'event':'stage_feedback','name':'descend','rows':[row]}
        self.feed('arm_feedback',event);self.feed('arm_feedback',event)
        self.assertEqual(sum('六轴编码' in v for v in self.lines),1)
        self.assertIn('+2',self.lines[-1])
        self.clock.sleep(1);self.feed('arm_feedback',event)
        self.assertEqual(sum('六轴编码' in v for v in self.lines),2)
    def test_failure_and_traceback_remain_visible(self):
        self.feed('pick_failed_no_extra_motion','No reliable physical blue20mm face')
        self.assertIn('No reliable',self.lines[-1])
        self.display.feed(b'Traceback (most recent call last):\nValueError: failed\n')
        self.assertIn('ValueError',self.lines[-1])
        self.feed('voice_failed','microphone unavailable')
        self.assertIn('当前不能唤醒',self.lines[-1])
    def test_malformed_event_does_not_hide_following_event(self):
        self.display.feed(b'AIC_PICK {broken\n')
        self.assertIn('解析失败',self.lines[-1])
        self.feed('voice_ready');self.assertIn('监听就绪',self.lines[-1])
    def test_silence_never_invents_running_or_listening_state(self):
        self.display.heartbeat()
        self.assertIn('静默不能说明',self.lines[-1])
        self.assertNotIn('监听就绪',self.lines[-1])
        self.feed('voice_ready');self.clock.sleep(30);self.display.heartbeat()
        self.assertIn('最近事件',self.lines[-1]);self.assertIn('30秒前',self.lines[-1])
        self.feed('voice_listening',{'chunks_read':32,'audio_ms':5120})
        self.assertIn('音频采集与识别循环正在运行',self.lines[-1])
    def test_localized_and_completed_do_not_claim_physical_grasp_success(self):
        self.feed('localized',{'cube_center_Base_mm':[100,20,-50]})
        self.assertIn('[100.00, 20.00, -50.00]',self.lines[-1])
        self.feed('complete_torque_on')
        self.assertIn('需观察',self.lines[-1])

class MonitorProtocolTests(unittest.TestCase):
    def test_receiver_opens_raw_cdc_without_sending_any_bytes(self):
        class Port:
            def write(self,data):raise AssertionError('No transmitted bytes allowed')
        with patch('host.apps.k230_status.serial.Serial',return_value=Port()) as serial:
            self.assertIsInstance(open_receiver('/fake/canmv'),Port)
        serial.assert_called_once_with('/fake/canmv',921600,timeout=.1,exclusive=True)
    def test_monitor_only_reads_even_on_exit(self):
        clock=Clock();closed=[];chunks=[b'AIC_PICK {"event":"voice_ready"}\n']
        class Port:
            port='/fake/canmv'
            @property
            def in_waiting(self):return len(chunks[0]) if chunks else 0
            def write(self,data):raise AssertionError('No transmitted bytes allowed')
            def read(self,size):return chunks.pop(0) if chunks else b''
            def close(self):closed.append(True)
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()):
            monitor('/fake',seconds=.3,output=folder,receiver_factory=lambda p:Port(),clock=clock,sleep=clock.sleep)
            self.assertIn('监听就绪',(Path(folder)/'status.log').read_text())
            self.assertIn(b'voice_ready',(Path(folder)/'board.log').read_bytes())
        self.assertEqual(closed,[True])
    def test_keyboard_interrupt_closes_port_without_stop_command(self):
        closed=[];port=SimpleNamespace(in_waiting=0,read=lambda n:b'',close=lambda:closed.append(True))
        def sleep(s):raise KeyboardInterrupt()
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()):
            monitor('/fake',output=folder,receiver_factory=lambda p:port,sleep=sleep)
        self.assertEqual(closed,[True])
    def test_missing_connection_is_reported_without_stopping_device(self):
        clock=Clock()
        def connect(p):raise OSError('device absent')
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()):
            monitor('/fake',seconds=.5,output=folder,receiver_factory=connect,clock=clock,sleep=clock.sleep)
            self.assertIn('不能据此判断',(Path(folder)/'status.log').read_text())
    def test_reconnect_does_not_join_fragments_from_two_connections(self):
        clock=Clock();ports=[]
        class Port:
            in_waiting=64
            def __init__(self,chunks):self.chunks=chunks;self.closed=False
            def read(self,n):
                data=self.chunks.pop(0) if self.chunks else b''
                if isinstance(data,Exception):raise data
                return data
            def close(self):self.closed=True
        def connect(p):
            port=Port([b'AIC_PICK {"event":',OSError('unplugged')] if not ports else [b'AIC_PICK {"event":"voice_ready"}\n'])
            ports.append(port);return port
        with tempfile.TemporaryDirectory() as folder,contextlib.redirect_stdout(io.StringIO()):
            monitor('/fake',seconds=1.6,output=folder,receiver_factory=connect,clock=clock,sleep=clock.sleep)
            logs=(Path(folder)/'status.log').read_text()
            self.assertIn('监听就绪',logs);self.assertNotIn('解析失败',logs)
        self.assertEqual(len(ports),2);self.assertTrue(all(p.closed for p in ports))

if __name__=='__main__':unittest.main()
