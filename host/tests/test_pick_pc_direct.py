"""PC invokes the existing standalone stage policy; no hardware or camera."""
import ast
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from host.apps import pick_validation_pc as app
from host.project_paths import ROOT
from host.transport import k230_pick_stage as transport

PROFILE = ROOT/'configs/arm/pick_model_check.json'


def inner_source(wrapped):
    return next(n.args[0].value for n in ast.walk(ast.parse(wrapped))
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id == 'exec')


class PcDirectTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pick, cls.config = app.parameters(PROFILE)
        cls.payload = transport.payload(cls.config, [2048]*6)

    def run_generated(self, name, values=()):
        stage = {'name': name, 'duration_s': 1,
                 'policy': self.pick['standalone_stage_policy']}
        source = ast.parse(inner_source(transport.build_action(self.payload, 'stage_direct', stage)))
        # Execute all actual helper definitions, then stub only the physical
        # UART replies before executing the generated direct-call suffix.
        clock = [0]
        writes, reads, closed = [], [], []
        values = iter(values)
        class Pins:
            UART1_TXD = 1
            UART1_RXD = 2
            def set_function(self, *args): pass
        class Port:
            UART1 = 1
            EIGHTBITS = 8
            PARITY_NONE = None
            STOPBITS_ONE = 1
            def __init__(self, *args, **kwargs): pass
            def write(self, packet):
                writes.append((clock[0], packet))
                return len(packet)
            def deinit(self): closed.append(True)
        def sleep(ms): clock[0] += ms
        def read(unused, servo_id, address, size):
            self.assertEqual((address, size), (56, 2))
            reads.append(servo_id)
            clock[0] += 30
            value = next(values)
            return bytes((value&255, value>>8))
        def forbidden(*args, **kwargs):
            raise AssertionError('Direct PC stage must not run legacy preflight/hold/dispatch')
        ns = {}
        imports = {'machine': SimpleNamespace(FPIOA=Pins, UART=Port), 'ujson': json,
                   'time': SimpleNamespace(ticks_ms=lambda: clock[0],
                       ticks_diff=lambda a,b: a-b, sleep_ms=sleep)}
        with patch.dict(sys.modules, imports):
            exec(compile(ast.Module(body=source.body[:-2], type_ignores=[]), '<actual stage helpers>', 'exec'), ns)
        ns.update(read_checked=read, preflight=forbidden,
                  initialize_hold=forbidden, send_sync=forbidden)
        with contextlib.redirect_stdout(io.StringIO()) as output:
            exec(compile(ast.Module(body=source.body[-2:], type_ignores=[]), '<generated direct call>', 'exec'), ns)
        events = [json.loads(line.split(' ', 1)[1]) for line in output.getvalue().splitlines()
                  if line.startswith('PICK_EVENT ')]
        self.assertIn('PICK_ACTION_PASS stage_direct', output.getvalue())
        self.assertEqual(closed, [True])
        return SimpleNamespace(writes=writes, reads=reads, elapsed=clock[0], events=events)

    def test_generated_hover_uses_fresh_all_axis_30_count_gate(self):
        result = self.run_generated('hover', [2079]+[2048]*5+[2018,2078,2048,2048,2048,2078])
        self.assertEqual(result.reads, list(range(1,7))*2)
        gate = next(e['details'] for e in result.events if e['event']=='position_gate_passed')
        self.assertEqual(gate['positions'], [2018,2078,2048,2048,2048,2078])
        self.assertEqual(gate['tolerance_counts'], 30)
        self.assertLess(result.elapsed, 6000)
        self.assertEqual(len(result.writes), 1)

    def test_generated_descend_reuses_200ms_axis4_lead_and_10_count_gate(self):
        result = self.run_generated('descend', [2059]+[2048]*5+[2038,2058,2048,2048,2048,2058])
        self.assertEqual([list(packet[7:-1:8]) for _, packet in result.writes], [[4], [1,2,3,5,6]])
        self.assertEqual(result.writes[1][0]-result.writes[0][0], 200)
        gate = next(e['details'] for e in result.events if e['event']=='position_gate_passed')
        self.assertEqual(gate['tolerance_counts'], 10)
        self.assertEqual(result.reads, list(range(1,7))*2)

    def test_generated_close_reads_only_6_and_accepts_exact_five_sample_band(self):
        result = self.run_generated('close', [2200,2201,2199,2200,2201])
        self.assertEqual(result.reads, [6]*5)
        settled = next(e['details'] for e in result.events if e['event']=='gripper_settled')
        self.assertEqual(settled['sample_count'], 5)
        self.assertEqual(settled['tolerance_counts'], 1)
        self.assertEqual(settled['positions'], [2200,2201,2199,2200,2201])
        self.assertLess(result.elapsed, 500)

    def test_generated_close_reports_signed_delta_threshold_without_extra_samples(self):
        for delta,confirmed in ((29,False),(30,True)):
            with self.subTest(delta=delta):
                result=self.run_generated('close',[2048+delta]*5)
                self.assertEqual(result.reads,[6]*5)
                self.assertEqual(len(result.writes),1)
                self.assertLess(result.elapsed,500)
                report=next(event['details'] for event in result.events
                    if event['event']=='grip_confirmation')
                self.assertEqual(report['goal'],2048)
                self.assertEqual(report['position'],2048+delta)
                self.assertEqual(report['delta_counts'],delta)
                self.assertEqual(report['threshold_counts'],30)
                self.assertIs(report['grasp_confirmed'],confirmed)

    def test_generated_lift_waits_one_second_without_extra_position_reads(self):
        result = self.run_generated('lift')
        self.assertEqual(result.reads, [])
        self.assertEqual(result.elapsed, 1002)
        self.assertEqual(result.events[-1],
                         {'event':'stage_complete', 'name':'lift', 'arrival_checked':False})

    def test_legacy_stage_read_and_hold_still_dispatch_original_action(self):
        for action in ('stage', 'read', 'hold'):
            with self.subTest(action=action):
                source = ast.parse(inner_source(transport.build_action(
                    self.payload, action, {'name':'hover', 'duration_s':6})))
                call = source.body[-1].value
                self.assertEqual(call.func.id, 'run_pick_action')
                self.assertEqual(call.args[1].value, action)

    def test_direct_position_gates_have_no_host_deadline_and_legacy_timeout_is_unchanged(self):
        executions = []
        class Link:
            def __init__(self, port):
                self.port = SimpleNamespace(close=lambda: None)
            def execute(self, source, timeout):
                executions.append(timeout)
                action = 'stage_direct' if 'PICK_ACTION_PASS stage_direct' in inner_source(source) else 'stage'
                yield ('PICK_ACTION_PASS '+action).encode()
        cases = [('stage_direct','hover', math.inf), ('stage_direct','descend', math.inf),
                 ('stage_direct','close', math.inf), ('stage_direct','lift',45),
                 ('stage','hover',45), ('stage','close',86400)]
        with patch.object(transport, 'CameraLink', Link):
            for action, name, expected in cases:
                with self.subTest(action=action, name=name):
                    transport.run_action(self.payload, action,
                        {'name':name, 'policy':self.pick['standalone_stage_policy']})
                    self.assertEqual(executions[-1], expected)


class PcDirectFlowTests(unittest.TestCase):
    def exercise(self, missing_gate=None, fail_stage=None, grip_delta=30):
        pick, config = app.parameters(PROFILE)
        plan = {name: [2200+i]*6 for i,name in enumerate(('hover','descend','close','lift'))}
        plan['table_z_Base_mm'] = -53
        actual = {'hover':[2230,2170,2200,2200,2200,2200],
                  'descend':[2211,2191,2201,2201,2201,2201]}
        calls, reports = [], []
        def report(counts):
            reports.append(list(counts))
            return {'fixture_observed_positions':list(counts)}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            def perceive(out, pick_config=None, port=None):
                self.assertEqual(pick_config,PROFILE)
                return {'status':'localized','cube_center_Base_mm':[245,-28,-65],
                    'xy_compensation_applied':False}
            def action(payload, kind, stage=None, **kwargs):
                name = stage['name'] if stage else kind
                calls.append((kind, name))
                if kind=='read': return [{'rows':[{'position':2048}]*6}]
                if kind=='hold': return []
                self.assertEqual(kind, 'stage_direct')
                self.assertEqual(stage['policy'], pick['standalone_stage_policy'])
                self.assertEqual(payload['goals'], plan[name])
                self.assertEqual(payload['profiles'], [config.profiles[i] for i in range(1,7)])
                if name==fail_stage: raise RuntimeError('missing feedback fixture')
                events = [{'event':'stage_started','name':name, 'packet_count':2 if name=='descend' else 1,
                           'dispatch_groups':[[4],[1,2,3,5,6]] if name=='descend' else [[1,2,3,4,5,6]],
                           'axis4_lead_ms':200 if name=='descend' else None}]
                if name in actual and name!=missing_gate:
                    positions = actual[name]
                    events.append({'event':'position_gate_passed','name':name,'details':{
                        'goals':plan[name], 'positions':positions,
                        'errors_actual_minus_goal':[v-g for v,g in zip(positions, plan[name])],
                        'tolerance_counts':30 if name=='hover' else 10, 'source':'arrival_gate'}})
                if name=='close':
                    position=plan[name][5]+(grip_delta or 0)
                    events.append({'event':'gripper_settled','details':{
                        'sample_count':5,'tolerance_counts':1,'positions':[position]*5}})
                    if grip_delta is not None:
                        events.append({'event':'grip_confirmation','details':{
                            'axis':6,'goal':plan[name][5],'position':position,
                            'delta_counts':grip_delta,'threshold_counts':30,
                            'grasp_confirmed':grip_delta>=30,
                            'method':'axis6_stable_goal_delta','position_reference':'last_stable_sample'}})
                events.append({'event':'stage_complete','name':name,'arrival_checked':name in actual})
                for event in events: kwargs['emit'](event)
                return events
            output = io.StringIO()
            with patch.object(app, 'ROOT', root), patch.object(app, 'parameters', return_value=(pick, config)),\
                 patch.object(app, 'perceive_onboard', side_effect=perceive),\
                 patch.object(app.subprocess, 'run', side_effect=AssertionError('live perception must stay on K230')),\
                 patch.object(app, 'make_plan', return_value=(SimpleNamespace(report=report), plan)),\
                 patch.object(app, 'run_action', side_effect=action), patch.object(app.signal, 'signal'),\
                 contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
                result = app.main(['--pick-config', str(PROFILE), '--execute', '--hands-free'])
            record = json.loads(next(root.glob('data/logs/*/execution.json')).read_text())
            recorded_events=[json.loads(line) for line in next(root.glob('data/logs/*/events.jsonl')).read_text().splitlines()]
        return SimpleNamespace(code=result, calls=calls, reports=reports, record=record,
                               observed=actual, output=output.getvalue(), events=recorded_events)

    def test_corrected_profile_uses_direct_stages_and_only_observed_gate_feedback_for_fk(self):
        result = self.exercise()
        self.assertEqual(result.code, 0)
        self.assertEqual(result.calls, [('read','read'),('hold','hold')]+
            [('stage_direct',s) for s in ('hover','descend','close','lift')])
        self.assertEqual(result.reports, [result.observed['hover'],result.observed['descend']])
        for stage in result.record['stages'][:2]:
            self.assertEqual(stage['feedback_source'], 'position_gate_passed')
            self.assertEqual([r['position'] for r in stage['final_feedback']], result.observed[stage['name']])
            self.assertEqual(stage['final_error_counts'],
                [v-g for v,g in zip(result.observed[stage['name']],stage['goals'])])
            self.assertEqual(stage['final_FK']['fixture_observed_positions'],result.observed[stage['name']])
        for stage in result.record['stages'][2:]:
            self.assertIsNone(stage['final_feedback'])
            self.assertIsNone(stage['final_error_counts'])
            self.assertIsNone(stage['final_FK'])
            self.assertIn('unavailable',stage['feedback_source'])
        self.assertEqual(result.record['descend_position_sample']['positions'],result.observed['descend'])
        self.assertFalse(result.record['pid_writes_sent'])
        self.assertEqual(result.record['pick_profile']['cube_offset_toward_moving_finger_mm'], 0)
        self.assertIn('4号已提前200毫秒发送',result.output)
        self.assertIn('连续5次处于±1计数',result.output)
        self.assertNotIn('连续0.5秒',result.output)

    def test_grip_threshold_results_are_logged_without_skipping_lift_or_forging_human_labels(self):
        for delta,confirmed in ((29,False),(30,True)):
            with self.subTest(delta=delta):
                result=self.exercise(grip_delta=delta)
                self.assertEqual(result.code,0)
                self.assertEqual(result.calls,[('read','read'),('hold','hold')]+
                    [('stage_direct',name) for name in ('hover','descend','close','lift')])
                self.assertIs(result.record['grasp_confirmed'],confirmed)
                confirmation=result.record['grip_confirmation']
                self.assertEqual(confirmation['goal'],result.record['plan']['close'][5])
                self.assertEqual(confirmation['delta_counts'],delta)
                self.assertEqual(result.record['grasp_confirmation_method'],'axis6_stable_goal_delta')
                self.assertEqual(result.record['stages'][2]['grip_confirmation'],confirmation)
                self.assertIs(result.record['stages'][2]['grasp_confirmed'],confirmed)
                self.assertEqual(result.record['physical_grasp_success'],'operator_observation_required')
                self.assertEqual(result.record['status'],'motion_sequence_complete_torque_on')
                stored=next(event['details'] for event in result.events if event['event']=='grip_confirmation')
                self.assertEqual(stored,confirmation)
                self.assertIn('Δ=%+d'%delta,result.output)
                self.assertIn('门槛30计数',result.output)
                self.assertIn('编码器规则判定：'+('成功' if confirmed else '未夹住'),result.output)
                self.assertEqual(sum(kind=='read' for kind,name in result.calls),1)

    def test_missing_grip_confirmation_does_not_manufacture_a_result(self):
        result=self.exercise(grip_delta=None)
        self.assertEqual(result.code,0)
        self.assertIsNone(result.record['grasp_confirmed'])
        self.assertNotIn('grip_confirmation',result.record)
        self.assertEqual(result.record['physical_grasp_success'],'operator_observation_required')
        self.assertIn('夹持判定不可用',result.output)
        self.assertEqual(result.calls[-1],('stage_direct','lift'))

    def test_missing_position_gate_cannot_dispatch_next_stage(self):
        result = self.exercise(missing_gate='hover')
        self.assertEqual(result.code, 1)
        self.assertEqual(result.calls[-1], ('stage_direct','hover'))
        self.assertEqual(result.record['stages'], [])
        self.assertEqual(result.reports, [])

    def test_failed_close_leaves_lift_undispatched_without_extra_reads(self):
        result = self.exercise(fail_stage='close')
        self.assertEqual(result.code, 1)
        self.assertEqual(result.calls[-1], ('stage_direct','close'))
        self.assertEqual(len(result.record['stages']), 2)
        self.assertEqual([name for kind,name in result.calls if kind=='read'], ['read'])


if __name__ == '__main__':
    unittest.main()
