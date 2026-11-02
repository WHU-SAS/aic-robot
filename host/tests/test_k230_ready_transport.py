"""K230 ready packets must match the existing STS SDK and inherited limits."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from host.project_paths import bootstrap
bootstrap()
from scservo_sdk import sms_sts,COMM_SUCCESS
from tools.diagnostics.k230_arm_ready import config_payload

class K230ReadyTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('board_ready',ROOT/'device/k230/arm/ready_pose.py')
        self.board=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'machine':SimpleNamespace(UART=None,FPIOA=None),'ujson':json}):
            spec.loader.exec_module(self.board)
        self.config=config_payload()
    def test_sync_payload_matches_existing_sdk_for_both_poses(self):
        for pose,expected in ((1,[2048,1081,3015,3015,2048,2332]),
                              (2,[2048,1138,2958,2958,2048,2332])):
            with self.subTest(pose=pose):
                config=config_payload(pose);sdk=sms_sts(None);captured=[]
                def tx(address,size,params,param_length):
                    captured.append((address,size,list(params),param_length));return COMM_SUCCESS
                sdk.syncWriteTxOnly=tx
                for i in range(1,7):
                    p=config['profiles'][i-1]
                    self.assertTrue(sdk.SyncWritePosEx(i,config['goals'][i-1],p['speed_raw'],p['acceleration_raw']))
                sdk.groupSyncWrite.txPacket()
                address,size,params,length=captured[0]
                actual=self.board.sync_frame(config['goals'],config['profiles'])
                self.assertEqual(actual[:5],bytes((255,255,254,length+4,0x83)))
                self.assertEqual(actual[5:-1],bytes([address,size]+params))
                self.assertEqual(sum(actual[2:])&255,255)
                self.assertEqual(config['goals'],expected)
                self.assertEqual(config['ready_pose_number'],pose)
                self.assertEqual(config['gripper_opens_deg'],25)
    def test_ready_returns_after_one_packet_without_post_dispatch_monitoring(self):
        events=[]
        class Pins:
            UART1_TXD=1;UART1_RXD=2
            def set_function(self,*args):pass
        class Port:
            UART1=1;EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=1
            def __init__(self,*args,**kwargs):pass
            def write(self,data):events.append(('packet',data));return len(data)
            def deinit(self):events.append('closed')
        def preflight(uart,config):
            events.append('preflight');return [{'torque':1} for unused in range(6)]
        with patch.object(self.board,'FPIOA',Pins),patch.object(self.board,'UART',Port),\
             patch.object(self.board,'preflight',preflight),\
             patch.object(self.board,'read_checked',side_effect=lambda u,i,a,n:events.append(('torque_read',i)) or bytes((1,))),\
             patch.object(self.board.time,'ticks_ms',create=True,return_value=0),\
             patch.object(self.board.time,'ticks_diff',create=True,side_effect=lambda a,b:a-b),\
             patch.object(self.board,'feedback',side_effect=AssertionError('No post-send feedback')),\
             patch.object(self.board,'snapshot',side_effect=AssertionError('No monitoring')),\
             patch.object(self.board.time,'sleep_ms',create=True,side_effect=lambda ms:events.append(('sleep_ms',ms))):
            self.board.run_arm_ready(self.config,'ready')
        self.assertEqual(events,['preflight']+[('torque_read',i) for i in range(1,7)]+[('packet',self.board.sync_frame(
            self.config['goals'],self.config['profiles'])),('sleep_ms',2),'closed'])

    def test_enable_only_one_byte_sram(self):
        packet=self.board.frame(2,3,(40,1))
        self.assertEqual(packet,bytes((255,255,2,4,3,40,1,205)))
    def test_preflight_outside_device_limits_never_writes(self):
        def read(uart,i,address,size):
            if address==3:return bytes((9,3,i,0))
            if address==18:
                data=[0]*10;data[0]=12
                for k,v in self.config['registers'][i-1].items():data[int(k)-18]=v
                return bytes(data)
            if address==9:return bytes((0,4,0,12))
            if address==33:return bytes((0,))
            if address==55:return bytes((1,))
            raise AssertionError(address)
        self.board.read_checked=read
        self.board.feedback=lambda uart,i:{'position':992 if i==2 else 2048,'torque':0}
        uart=SimpleNamespace(write=lambda data:self.fail('No write during preflight'))
        with self.assertRaisesRegex(RuntimeError,'ID2 position992'):
            self.board.preflight(uart,self.config)
    def test_mismatched_pid_never_writes(self):
        def read(uart,i,address,size):
            if address==3:return bytes((9,3,i,0))
            if address==18:return bytes((12,0,0,0,0,0,0,0,0,0))
            if address==9:return bytes((0,4,0,12))
            if address==33:return bytes((0,))
            if address==55:return bytes((1,))
            raise AssertionError(address)
        self.board.read_checked=read
        with self.assertRaisesRegex(RuntimeError,'baseline register21'):
            self.board.preflight(None,self.config)
