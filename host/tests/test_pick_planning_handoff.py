"""IK handoff yields observation opportunities without taking motion ownership."""
import copy
from contextlib import ExitStack, contextmanager
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from host.tests.test_pick_handoff import load_app, observation, car_source
from tools.deploy.k230_pick import build_bundle
from device.k230.pick.kinematics import Geometry, NoPickSolution


class PickPlanningHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle, _, _ = build_bundle()

    def setUp(self):
        self.app = load_app()
        self.config = copy.deepcopy(self.bundle)
        self.source = car_source(self.config)
        self.observation = observation()
        self.start = [2043, 2100, 2250, 1950, 2048, 2389]
        self.ready_counts = list(self.config['drive_ready_control']['goals'])
        self.reference = {'source': 'ready_pose2_command',
                          'counts': list(self.ready_counts), 'measured': False}
        self.plan = {name: [2100 + i*10 + j for j in range(6)]
                     for i, name in enumerate(('hover','descend','close','lift'))}
        self.events = []

    @contextmanager
    def harness(self):
        def snapshot(config):
            raise AssertionError('Planning uses commanded ready2, never arm_snapshot')
        def planning(*args):
            self.events.append('planning_started')
            try:
                yield {'stage':'tilt_candidate','candidate':25,'target_version':999}
                yield {'stage':'complete','plan':copy.deepcopy(self.plan)}
            finally:self.events.append('planning_closed')
        geometry = Mock()
        geometry.plan_steps.side_effect = planning
        def construct(*args):
            self.events.append('geometry')
            return geometry
        execution = Mock(name='inert_execution')
        def factory(*args, **kwargs):
            self.events.append('factory')
            return execution
        with ExitStack() as stack:
            create = Mock(side_effect=factory)
            stack.enter_context(patch.dict(sys.modules, {
                'control':SimpleNamespace(open_pick_execution=create)}))
            stack.enter_context(patch.object(self.app._kinematics,
                'NoPickSolution',NoPickSolution,create=True))
            read = stack.enter_context(patch.object(self.app,'arm_snapshot',side_effect=snapshot))
            constructor = stack.enter_context(patch.object(self.app,'Geometry',side_effect=construct))
            logs = stack.enter_context(patch.object(self.app,'log'))
            forbidden = {}
            for name in ('CameraSession','localize','localize_precise','perceive',
                         'run_pick_stage_direct','run_ready_direct','run_ready'):
                forbidden[name] = stack.enter_context(patch.object(self.app,name,
                    side_effect=AssertionError('Planning may not capture, dispatch or recover')))
            yield SimpleNamespace(factory=create,execution=execution,geometry=geometry,
                read=read,constructor=constructor,logs=logs,forbidden=forbidden)

    def begin(self, **kwargs):
        return self.app.begin_from_observation_steps(self.config,self.observation,17,
            source_config=self.source,**kwargs)

    def test_construction_is_lazy_then_reference_setup_and_search_yield_separately(self):
        with self.harness() as h:
            job = self.begin()
            self.assertEqual(self.events,[])
            first = next(job)
            self.assertEqual(first,{'stage':'planning_reference','target_version':17,
                'frame_id':71,'planning_reference':self.reference})
            self.assertNotIn('positions',first)
            self.assertEqual(self.events,[])
            second = next(job)
            self.assertEqual(second,{'stage':'planning_initialized','target_version':17,
                'frame_id':71})
            self.assertEqual(self.events,['geometry'])
            progress = next(job)
            self.assertEqual(progress,{'stage':'tilt_candidate','candidate':25,'target_version':17,'frame_id':71})
            self.assertEqual(self.events,['geometry','planning_started'])
            h.factory.assert_not_called()
            complete = next(job)
            self.assertEqual(complete,{'stage':'complete','execution':h.execution,
                'target_version':17,'frame_id':71})
            self.assertEqual(self.events,['geometry','planning_started','planning_closed','factory'])
            with self.assertRaises(StopIteration):next(job)
            h.read.assert_not_called()
            h.geometry.plan_steps.assert_called_once_with([.245,-.028],self.ready_counts)
            h.geometry.plan.assert_not_called()
            h.execution.step.assert_not_called()
            h.execution.close.assert_not_called()
            for callback in h.forbidden.values():callback.assert_not_called()

    def test_factory_receives_same_ready_pose2_and_merged_policy_as_blocking_begin(self):
        emit = Mock()
        with self.harness() as h:
            complete = list(self.begin(emit=emit))[-1]
        args = h.factory.call_args.args
        self.assertIs(args[0],self.config['control'])
        self.assertIs(args[1],self.config['drive_ready_control'])
        self.assertEqual(args[2],dict(self.plan,planning_reference=self.reference))
        self.assertIsNot(args[2]['planning_reference']['counts'],self.config['drive_ready_control']['goals'])
        self.assertEqual(args[3],dict(self.config['pick']['standalone_stage_policy'],
            **self.config['pick'].get('tracking_stage_policy',{})))
        self.assertEqual(args[4],17)
        self.assertIs(h.factory.call_args.kwargs['emit'],emit)
        self.assertIs(complete['execution'],h.execution)

    def test_initial_and_recovery_step_plans_share_ready2_seed_without_read_or_ready_command(self):
        for version, frame_id, actual_pose in (
                (17,71,[2048,1200,2900,2800,2048,2332]),
                (18,72,[1900,2600,2500,2300,1800,2200])):
            with self.subTest(version=version):
                self.start=actual_pose
                self.observation['frame_id']=frame_id
                with self.harness() as h:
                    events=list(self.app.begin_from_observation_steps(self.config,
                        self.observation,version,source_config=self.source))
                self.assertEqual(events[0],{'stage':'planning_reference',
                    'planning_reference':self.reference,'target_version':version,'frame_id':frame_id})
                self.assertTrue(all(event['target_version']==version for event in events))
                self.assertTrue(all(event['frame_id']==frame_id for event in events))
                h.geometry.plan_steps.assert_called_once_with([.245,-.028],self.ready_counts)
                h.read.assert_not_called()
                self.assertEqual(h.factory.call_args.args[2]['planning_reference'],self.reference)
                h.execution.step.assert_not_called()
                for callback in h.forbidden.values():callback.assert_not_called()

    def test_default_event_callback_uses_existing_log_without_serializing_execution(self):
        with self.harness() as h:
            list(self.begin())
            h.factory.call_args.kwargs['emit']('status',{'phase':'hover'})
            h.logs.assert_any_call('status',{'phase':'hover'})
            h.logs.assert_any_call('planned',dict(self.plan,planning_reference=self.reference))
            for call in h.logs.call_args_list:
                if len(call.args)>1 and isinstance(call.args[1],dict):
                    self.assertNotIn('execution',call.args[1])

    def test_bound_version_and_original_frame_survive_mutated_observation_container(self):
        with self.harness() as h:
            job = self.begin()
            next(job)
            self.observation['frame_id'] = 999
            self.observation['capture_timestamp_ms'] = 777
            self.observation['cube_center_Base_mm'][0] = 9999.
            remaining = list(job)
            self.assertTrue(all(event['target_version']==17 for event in remaining))
            self.assertEqual(remaining[-1]['frame_id'],71)
            h.geometry.plan_steps.assert_called_once_with([.245,-.028],self.ready_counts)
            h.logs.assert_any_call('tracking_handoff_localized',{
                'frame_id':71,'capture_timestamp_ms':5,
                'cube_center_Base_mm':[245.,-28.,-65.],'target_version':17,
                'source':'accepted_static_observation'})

    def test_cancel_before_first_next_is_inert(self):
        with self.harness() as h:
            job = self.begin()
            job.close()
            h.read.assert_not_called()
            h.constructor.assert_not_called()
            h.factory.assert_not_called()

    def test_cancel_at_each_progress_point_never_creates_execution_or_recovers(self):
        for count in (1,2,3):
            with self.subTest(count=count),self.harness() as h:
                self.events=[]
                job=self.begin()
                for _ in range(count):next(job)
                job.close()
                with self.assertRaises(StopIteration):next(job)
                h.factory.assert_not_called()
                if count==3:self.assertIn('planning_closed',self.events)
                for callback in h.forbidden.values():callback.assert_not_called()

    def test_closing_after_final_handoff_leaves_execution_owned_by_caller(self):
        with self.harness() as h:
            job=self.begin()
            for _ in range(3):next(job)
            self.assertIs(next(job)['execution'],h.execution)
            job.close()
            h.execution.close.assert_not_called()

    def test_pre_handoff_error_after_factory_cleans_up_inert_execution(self):
        with self.harness() as h:
            def log(event,details=None):
                if event=='tracking_handoff_localized':raise OSError('Log resources lost')
            h.logs.side_effect=log
            with self.assertRaisesRegex(OSError,'Log resources lost'):list(self.begin())
            h.execution.close.assert_called_once_with()
            h.execution.step.assert_not_called()
            for callback in h.forbidden.values():callback.assert_not_called()

    def test_math_no_solution_on_create_or_next_is_the_only_recoverable_type(self):
        for where in ('create','next'):
            with self.subTest(where=where),self.harness() as h:
                def failed(*args):
                    yield {'stage':'tilt_candidate'}
                    raise NoPickSolution('No complete four-stage IK: scene')
                h.geometry.plan_steps.side_effect=(NoPickSolution('No complete four-stage IK: scene')
                    if where=='create' else failed)
                with self.assertRaises(self.app.PickPlanUnavailable):list(self.begin())
                h.factory.assert_not_called()
                for callback in h.forbidden.values():callback.assert_not_called()

    def test_plain_plan_constructor_and_factory_value_errors_are_not_reclassified(self):
        for where in ('create','next','constructor','factory'):
            with self.subTest(where=where),self.harness() as h:
                def failed(*args):
                    yield {'stage':'tilt_candidate'}
                    raise ValueError('Bad numerical input')
                if where=='create':h.geometry.plan_steps.side_effect=ValueError('Bad numerical input')
                elif where=='next':h.geometry.plan_steps.side_effect=failed
                elif where=='constructor':h.constructor.side_effect=ValueError('Bad model')
                else:h.factory.side_effect=ValueError('Bad policy')
                with self.assertRaises(ValueError) as caught:list(self.begin())
                self.assertIs(type(caught.exception),ValueError)

    def test_caller_thrown_typed_error_at_progress_is_not_a_scene_solver_error(self):
        with self.harness() as h:
            job=self.begin()
            for _ in range(3):next(job)
            with self.assertRaises(NoPickSolution) as caught:
                job.throw(NoPickSolution('Caller cancellation'))
            self.assertIs(type(caught.exception),NoPickSolution)
            self.assertIn('planning_closed',self.events)
            h.factory.assert_not_called()

    def test_invalid_calibration_and_observation_fail_before_any_read(self):
        for corrupt in ('calibration','observation'):
            with self.subTest(corrupt=corrupt),self.harness() as h:
                self.source=car_source(self.config)
                self.observation=observation()
                if corrupt=='calibration':self.source['intrinsics']['camera_matrix'][0][0]+=1
                else:self.observation['range_only']=True
                with self.assertRaises(ValueError) as caught:next(self.begin())
                self.assertIs(type(caught.exception),ValueError)
                h.read.assert_not_called()
                h.factory.assert_not_called()

    def test_missing_complete_is_protocol_failure_without_execution(self):
        with self.harness() as h:
            h.geometry.plan_steps.side_effect=lambda *args:iter([{'stage':'candidate'}])
            with self.assertRaisesRegex(RuntimeError,'without a complete plan'):list(self.begin())
            h.factory.assert_not_called()

    def test_wrist_orientation_is_prepared_once_before_search(self):
        self.config['pick']['wrist_alignment_enabled']=True
        orientation={'status':'estimated'}
        estimate=Mock(return_value=orientation)
        with self.harness() as h,patch.dict(sys.modules,{'cube_orientation':SimpleNamespace(estimate=estimate)}):
            job=self.begin()
            next(job)
            estimate.assert_not_called()
            next(job)
            estimate.assert_called_once()
            list(job)
            h.geometry.plan_steps.assert_called_once_with([.245,-.028],self.ready_counts,orientation)

    def test_real_geometry_step_plan_matches_existing_synchronous_geometry(self):
        with self.harness() as h,patch.object(self.app,'Geometry',Geometry):
            result=list(self.begin())[-1]
        actual=h.factory.call_args.args[2]
        plane=self.config['vision']['localization']['table_plane_Base_mm']
        n=plane['normal'];xy=[245.,-28.]
        table_z=(plane['offset_mm']-n[0]*xy[0]-n[1]*xy[1])/n[2]
        expected=Geometry(self.config['model'],self.config['pick'],table_z).plan([.245,-.028],self.ready_counts)
        expected['planning_reference']=copy.deepcopy(self.reference)
        self.assertEqual(actual,expected)
        self.assertIs(result['execution'],h.execution)


if __name__=='__main__':unittest.main()
