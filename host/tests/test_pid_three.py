"""Offline checks of the actual board runner and finite-window measurements."""
from contextlib import redirect_stdout
import io
import json
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from host.apps.pid_three import make_config, board_script, summarize


class FixedTrialTests(unittest.TestCase):
    def run_board(self, failed_axis=None):
        clock = SimpleNamespace(ms=0)
        def ticks():return clock.ms
        def sleep(ms):clock.ms+=ms
        timer=SimpleNamespace(ticks_ms=ticks,ticks_diff=lambda a,b:a-b,sleep_ms=sleep)
        class UART:
            UART1=1;EIGHTBITS=8;PARITY_NONE=0;STOPBITS_ONE=1
            def __init__(self,*a,**kw):
                self.frames=[];self.buffer=None;self.goals=[2048]*6;self.closed=False;device.append(self)
            def write(self,packet):
                packet=bytes(packet);self.frames.append(packet)
                if packet[4]==0x83:
                    self.goals=[packet[9+8*i]+256*packet[10+8*i] for i in range(6)]
                elif packet[4]==2:
                    sid=packet[2];address,size=packet[5:7]
                    if address==56:
                        p=self.goals[sid-1]+(11 if sid==3 else 0)
                        values=[p&255,p>>8,0,0]
                    elif address==21:values=[16,160,4,12,48,1,1]
                    else:raise AssertionError(address)
                    body=bytes([sid,size+2,16 if failed_axis==sid else 0]+values)
                    self.buffer=b'\xff\xff'+body+bytes([(~sum(body))&255])
                else:raise AssertionError('Unexpected write instruction')
                return len(packet)
            def read(self):
                clock.ms+=1;v=self.buffer;self.buffer=None;return v
            def deinit(self):self.closed=True
        class Pins:
            UART1_TXD=3;UART1_RXD=4
            def set_function(self,*a):pass
        device=[];out=io.StringIO();config=make_config(.7,.6)
        with patch.dict(sys.modules,{'machine':SimpleNamespace(UART=UART,FPIOA=Pins),'time':timer}),redirect_stdout(out):
            exec(board_script(config),{})
        self.assertNotIn('Traceback',out.getvalue())
        events=[json.loads(s[11:]) for s in out.getvalue().splitlines() if s.startswith('PID_SAMPLE ')]
        return config,device[0],events

    def test_one_final_packet_per_transition_and_no_settings_writes(self):
        config,uart,events=self.run_board()
        packets=[p for p in uart.frames if p[4]==0x83]
        self.assertEqual(len(packets),7)
        for packet,stage in zip(packets,config['stages']):
            self.assertEqual(packet[5:7],bytes([41,7]))
            self.assertEqual(sum(packet[2:])&255,255)
            self.assertEqual([packet[9+8*i]+256*packet[10+8*i] for i in range(6)],stage['goals'])
            self.assertEqual([packet[8+8*i] for i in range(6)],[5,20,20,20,5,5])
            self.assertEqual([packet[13+8*i]+256*packet[14+8*i] for i in range(6)],[150,600,600,600,150,150])
        self.assertEqual(set(p[4] for p in uart.frames),{2,0x83})
        self.assertEqual(events[-1]['event'],'complete')
        self.assertTrue(uart.closed)
        report=summarize(config,events)
        self.assertTrue(report['complete'])
        self.assertEqual(report['stages'][1]['axes'][2]['final_error'],11)
        self.assertIsNone(report['stages'][1]['axes'][2]['goal_settled_at_s'])

    def test_feedback_error_is_recorded_without_changing_sequence(self):
        config,uart,events=self.run_board(failed_axis=3)
        self.assertEqual(len([p for p in uart.frames if p[4]==0x83]),7)
        row=summarize(config,events)['stages'][1]['axes'][2]
        self.assertEqual(row['valid_samples'],0)
        self.assertGreater(row['missing_samples'],0)

    def summary_for(self,positions,goal=1000):
        config={'stages':[{'name':'action','goals':[goal]*6}]}
        events=[{'event':'sample','index':0,'rows':[[i*.1,p,0,None] for _ in range(6)]} for i,p in enumerate(positions)]
        return summarize(config,events)['stages'][0]['axes'][0]

    def test_static_error_is_not_called_goal_convergence(self):
        row=self.summary_for([1011]*20)
        self.assertEqual(row['stable_at_s'],0)
        self.assertEqual(row['correction_s'],0)
        self.assertEqual(row['tail_peak_to_peak'],0)
        self.assertIsNone(row['goal_settled_at_s'])

    def test_slow_monotonic_correction_excludes_main_travel(self):
        row=self.summary_for([900,940,970,988,991,994,997,999]+[1000]*10)
        self.assertAlmostEqual(row['correction_s'],.4)
        self.assertEqual(row['reversals_after_near'],0)

    def test_oscillation_and_unfinished_drift_are_not_stable(self):
        row=self.summary_for([996,1004]*10)
        self.assertIsNone(row['stable_at_s'])
        self.assertIsNone(row['correction_s'])
        self.assertEqual(row['tail_peak_to_peak'],8)
        self.assertGreater(row['reversals_after_near'],5)
        self.assertIsNone(self.summary_for(list(range(960,1000,2)))['stable_at_s'])

    def test_small_encoder_noise_is_not_a_direction_reversal(self):
        row=self.summary_for([999,1000,1001,1000]*5)
        self.assertEqual(row['reversals_after_near'],0)
        self.assertEqual(row['tail_peak_to_peak'],2)

    def test_default_targets_and_duration_stay_frozen(self):
        config=make_config()
        self.assertEqual(sum(s['seconds'] for s in config['stages']),29)
        self.assertEqual([s['goals'][5] for s in config['stages']],[2389]*7)
        self.assertEqual([a['counts'] for a in config['actions']],[[2061,2673,2871,2364,2046,2389],[2057,2819,2584,2504,2046,2389],[2031,2908,2501,2385,2046,2389]])


if __name__=='__main__':unittest.main()
