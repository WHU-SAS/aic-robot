"""Full PC trial orchestration and one-stage board dispatch, without hardware."""
import contextlib
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from host.project_paths import ROOT
from host.arm.blue_pick_settle import PositionSettled
from host.apps import pick_validation_pc as app
from host.transport import k230_pick_stage as transport
from host.vision.config import load_vision_config, table_height_at


class BoardStageTests(unittest.TestCase):
    def runner(self, torque=1, fail=False):
        clock=SimpleNamespace(ms=0)
        def sleep(ms):clock.ms+=ms
        uart=SimpleNamespace(deinit=lambda:events.append('deinit'))
        events=[]
        def feedback(unused,i):
            if fail:raise RuntimeError('read failure')
            return {'id':i,'position':2000 if clock.ms<150 else 2004,'torque':torque}
        class Pins:
            UART1_TXD=1;UART1_RXD=2
            def set_function(self,*args):pass
        class Uart:
            UART1=1;EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=1
            def __new__(cls,*args,**kwargs):return uart
        ns={'FPIOA':Pins,'UART':Uart,'time':SimpleNamespace(ticks_ms=lambda:clock.ms,
            ticks_diff=lambda a,b:a-b,sleep_ms=sleep),'ujson':json,'PositionSettled':PositionSettled,
            'preflight':lambda *args:[{'torque':torque} for i in range(6)],
            'initialize_hold':lambda *args:events.append('hold'),
            'send_sync':lambda uart,goals,config:events.append(('sync',goals)),
            'feedback':feedback}
        exec((ROOT/'device/k230/arm/pick_stage.py').read_text(),ns)
        return ns['run_pick_action'],clock,events

    def test_one_sync_packet_and_no_release_per_stage(self):
        run,clock,events=self.runner()
        with contextlib.redirect_stdout(io.StringIO()):
            run({'goals':[2048]*6},'stage',{'name':'descend','duration_s':.15})
        self.assertEqual(events,[('sync',[2048]*6),'deinit'])
        self.assertGreaterEqual(clock.ms,150)

    def test_close_waits_half_second_after_last_large_position_change(self):
        run,clock,events=self.runner()
        output=io.StringIO()
        with contextlib.redirect_stdout(output):
            run({'goals':[2048]*6},'stage',{'name':'close','stationary_s':.5,'tolerance_counts':1})
        self.assertGreaterEqual(clock.ms,650)
        settled=next(json.loads(line[11:]) for line in output.getvalue().splitlines()
                     if line.startswith('PICK_EVENT ') and 'gripper_settled' in line)
        self.assertGreaterEqual(settled['details']['duration_s'],.5)
        self.assertEqual(settled['details']['position_min'],2004)
        self.assertEqual(events,[('sync',[2048]*6),'deinit'])

    def test_read_and_hold_do_not_dispatch_stage_goals(self):
        run,clock,events=self.runner()
        with contextlib.redirect_stdout(io.StringIO()):
            run({'goals':[2048]*6},'read');run({'goals':[2048]*6},'hold')
        self.assertEqual(events,['deinit','hold','deinit'])

    def test_feedback_failure_keeps_current_goal_and_deinitializes(self):
        run,clock,events=self.runner(fail=True)
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(RuntimeError,'read failure'):
            run({'goals':[2048]*6},'stage',{'name':'lift','duration_s':8})
        self.assertEqual(events,[('sync',[2048]*6),'deinit'])

    def test_stage_requires_hold_before_goal_dispatch(self):
        run,clock,events=self.runner(torque=0)
        with self.assertRaisesRegex(RuntimeError,'hold torque'):
            run({'goals':[2048]*6},'stage',{'name':'lift','duration_s':8})
        self.assertEqual(events,['deinit'])


class TrialConfigTests(unittest.TestCase):
    def test_candidate_pair_and_zero_extension_saved_motion_defaults(self):
        pick,config=app.parameters()
        vision=load_vision_config(ROOT/pick['vision_config'])['localization']
        ext=json.loads((ROOT/vision['extrinsics']).read_text())
        self.assertEqual(ext['intrinsics_sha256'],hashlib.sha256((ROOT/vision['intrinsics']).read_bytes()).hexdigest())
        self.assertEqual(pick['forward_extension_mm'],0)
        self.assertEqual(pick['alignment_reference'],'open_fingertip_midpoint')
        self.assertEqual(pick['cube_offset_toward_moving_finger_mm'],-45)
        self.assertEqual([pick[k] for k in ('opening_count','descend_tip_height_mm','hover_tip_height_mm','lift_tip_height_mm')],[2389,15,80,80])
        self.assertEqual([config.profiles[i] for i in range(1,7)],
            [{'speed_raw':v,'acceleration_raw':a} for v,a in [(150,5),(600,20),(600,20),(600,20),(150,5),(150,5)]])
        self.assertNotEqual(vision['extrinsics'],load_vision_config()['localization']['extrinsics'])
        self.assertEqual(table_height_at({'table_z_Base_mm':-63},[100,50]),-63)
        n=vision['table_plane_Base_mm']['normal'];d=vision['table_plane_Base_mm']['offset_mm']
        self.assertAlmostEqual(sum(a*b for a,b in zip(n,[220,30,table_height_at(vision,[220,30])])),d)

    def test_transport_interrupt_stops_only_board_script_and_closes(self):
        commands=[]
        class Link:
            def __init__(self,port):self.port=SimpleNamespace(close=lambda:commands.append('close'))
            def execute(self,*args,**kwargs):raise KeyboardInterrupt()
            def command(self,code):commands.append(code)
        pick,config=app.parameters()
        with patch.object(transport,'CameraLink',Link):
            with self.assertRaises(KeyboardInterrupt):
                transport.run_action(transport.payload(config,[2048]*6),'read')
        self.assertEqual(commands,[0x06,'close'])

    def test_board_bundle_is_self_contained_without_pc_imports(self):
        import ast
        pick,config=app.parameters()
        tree=ast.parse(transport.build_action(transport.payload(config,[2048]*6),'read'))
        inner=next(n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call)
                   and isinstance(n.func,ast.Name) and n.func.id=='exec')
        compile(inner,'board_bundle','exec')
        imports=[]
        for node in ast.walk(ast.parse(inner)):
            if isinstance(node,ast.ImportFrom):imports.append(node.module or '')
            elif isinstance(node,ast.Import):imports.extend(a.name for a in node.names)
        self.assertFalse(any(n.startswith(('host','serial','numpy','scipy','cv2')) for n in imports))
        self.assertEqual(inner.count('class PositionSettled:'),1)
        self.assertIn('run_pick_action(ujson.loads(',inner)

    def test_offline_image_cannot_execute(self):
        with patch.object(app,'parameters',side_effect=AssertionError('must not reach hardware setup')):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                app.main(['--execute','--image','frame.jpg'])


class FlowTests(unittest.TestCase):
    def exercise(self,fail_stage=None,detect_only=False):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);calls=[]
            pick,config=app.parameters()
            plan={k:[2048]*6 for k in ('hover','descend','close','lift')}
            plan['table_z_Base_mm']=-66
            def capture(out,port):
                (out/'frame.jpg').write_bytes(b'offline fixture');return {}
            def localize(command,**kwargs):
                out=Path(command[command.index('--output')+1])
                (out/'vision_record.json').write_text(json.dumps({'cube_center_Base_mm':[220,30,-56]}))
            def action(config,kind,stage=None,**kwargs):
                name=stage['name'] if stage else kind;calls.append(name)
                if name==fail_stage:raise RuntimeError('injected failure')
                rows=[{'position':2048} for i in range(6)]
                if kind=='read':return [{'rows':rows}]
                if kind=='hold':return []
                kwargs['emit']({'event':'stage_started','name':name})
                return [{'event':'stage_complete','name':name,'rows':rows}]
            with patch.object(app,'ROOT',root),patch.object(app,'parameters',return_value=(pick,config)),\
                 patch.object(app,'capture',side_effect=capture),patch.object(app.subprocess,'run',side_effect=localize),\
                 patch.object(app,'run_action',side_effect=action),patch.object(app,'make_plan',return_value=(SimpleNamespace(report=lambda c:{}),plan)),\
                 patch.object(app.signal,'signal'),contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                code=app.main(['--detect-only'] if detect_only else ['--execute','--hands-free'])
            record=json.loads(next(root.glob('data/logs/*/execution.json')).read_text())
            return code,calls,record

    def test_complete_flow_order_preserves_torque_and_never_writes_pid(self):
        code,calls,record=self.exercise()
        self.assertEqual(code,0)
        self.assertEqual(calls,['read','hold','hover','descend','close','lift'])
        self.assertEqual(record['status'],'motion_sequence_complete_torque_on')
        self.assertFalse(record['pid_writes_sent'])
        self.assertEqual(len(record['stages']),4)

    def test_failed_close_cannot_dispatch_lift(self):
        code,calls,record=self.exercise(fail_stage='close')
        self.assertEqual(code,1)
        self.assertEqual(calls,['read','hold','hover','descend','close'])
        self.assertEqual(record['status'],'error_no_extra_motion')
        self.assertEqual(len(record['stages']),2)

    def test_detection_only_never_accesses_servo_bus(self):
        code,calls,record=self.exercise(detect_only=True)
        self.assertEqual((code,calls),(0,[]))
        self.assertFalse(record['motion_performed'])
        self.assertFalse(record['torque_enable_attempted'])


class OpenTipGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np
        from host.arm.pick_validation_geometry import OpenTipPickGeometry
        pick,cfg=app.parameters()
        cls.g=OpenTipPickGeometry(ROOT,cfg.mapping,table_z_m=-.0688,
            hover_tip_height_mm=80,descend_tip_height_mm=15,lift_tip_height_mm=80,
            opening_count=pick['opening_count'])
        cls.start=[2048]*6
        cls.target=np.array([.271462,.041016])
        cls.plan=cls.g.plan(cls.target,cls.start)

    def test_descending_open_tips_straddle_target(self):
        import numpy as np
        q=self.g.angles(self.plan['descend'])
        midpoint,points,lows,unused=self.g.metrics(q)
        self.assertLess(np.linalg.norm(midpoint[:2]-self.target),.002)
        a=points['wan2'][:2]-self.target;b=points['zhua'][:2]-self.target
        self.assertLess(np.dot(a,b),0)
        self.assertGreater(np.linalg.norm(a),.027)
        self.assertAlmostEqual(min(lows.values())-self.g.table_z_m,.015,delta=.002)
        self.assertEqual(self.plan['alignment_reference'],'open_fingertip_midpoint')

    def test_alignment_reference_stays_open_when_reporting_closed_feedback(self):
        import numpy as np
        q=self.g.angles(self.plan['descend']);closed=q.copy()
        closed[5]=self.g.mapping.count_to_radians(6,self.plan['close'][5])
        np.testing.assert_allclose(self.g.alignment_midpoint(q),self.g.alignment_midpoint(closed))
        self.assertEqual(self.plan['descend'][5],2389)
        self.assertEqual(self.plan['close'][:5],self.plan['descend'][:5])
        self.assertEqual(self.plan['closing_count'],self.plan['nominal_closing_count']-20)


    def test_negative_offset_is_exactly_45mm_toward_fixed_jaw_for_each_heading(self):
        import numpy as np
        from host.arm.pick_validation_geometry import OpenTipPickGeometry
        unused,cfg=app.parameters()
        shifted=OpenTipPickGeometry(ROOT,cfg.mapping,opening_count=2389,
                                    cube_offset_toward_moving_finger_mm=-45)
        for yaw in (-.4,0,.4):
            q=self.g.angles(self.plan['descend']);q[0]=yaw
            mid,points,unused_low,unused_t=self.g.open_reference(q)
            expected=points['wan2'][:2]-points['zhua'][:2]
            expected=expected/np.linalg.norm(expected)*.045
            actual=shifted.alignment_midpoint(q)-mid
            np.testing.assert_allclose(actual[:2],expected,atol=1e-12)
            self.assertAlmostEqual(actual[2],0)
        report=shifted.report(self.plan['descend'])
        self.assertEqual(report['cube_offset_toward_moving_finger_mm'],-45)
