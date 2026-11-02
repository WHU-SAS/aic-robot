"""Cooperative IK must preserve frozen goals and expose cancellation points.

The goal fixtures were recorded from the pre-cooperative Geometry/wrist solver
on the corrected model. No hardware, camera or saved-image data is required.
"""
import math
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from tools.deploy.k230_pick import build_bundle
from device.k230.arm import pick_execution
from device.k230.pick.cube_orientation import plane_basis
from device.k230.pick.kinematics import Geometry, NoPickSolution
from device.k230.pick.numeric import cross


STAGES = ('hover', 'descend', 'close', 'lift')
START = [2048] * 6
FROZEN_HELD = {
    'hover': [2067, 2496, 2789, 2622, 2048, 2332],
    'descend': [2067, 2666, 2884, 2358, 2048, 2332],
    'close': [2067, 2666, 2884, 2358, 2048, 2142],
    'lift': [2067, 2496, 2789, 2622, 2048, 2142],
}
FROZEN_LEFT = {
    'hover': [2129, 2453, 2854, 2600, 2048, 2332],
    'descend': [2129, 2631, 2949, 2328, 2048, 2332],
    'close': [2129, 2631, 2949, 2328, 2048, 2155],
    'lift': [2129, 2453, 2854, 2600, 2048, 2155],
}
FROZEN_ALIGNED = {
    'hover': [2067, 2702, 2376, 2943, 2065, 2332],
    'descend': [2067, 2924, 2293, 2861, 2065, 2332],
    'close': [2067, 2924, 2293, 2861, 2065, 2140],
    'lift': [2067, 2702, 2376, 2943, 2065, 2140],
}
FROZEN_MISSING_ORIENTATION = {
    'hover': [2063, 2462, 2830, 2615, 1707, 2332],
    'descend': [2063, 2634, 2931, 2343, 1707, 2332],
    'close': [2063, 2634, 2931, 2343, 1707, 2192],
    'lift': [2063, 2462, 2830, 2615, 1707, 2192],
}


class PickPlanningStepsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config, _, _ = build_bundle()
        cls.plane = cls.config['vision']['localization']['table_plane_Base_mm']

    def geometry(self, xy_mm=(248., 4.), enabled=False):
        pick = dict(self.config['pick'], wrist_alignment_enabled=enabled,
                    cube_offset_toward_moving_finger_mm=0)
        n = self.plane['normal']
        z = (self.plane['offset_mm'] - n[0] * xy_mm[0] - n[1] * xy_mm[1]) / n[2]
        return Geometry(self.config['model'], pick, z)

    def orientation(self, degrees):
        x, y, n = plane_basis(self.plane)
        angle = math.radians(degrees)
        axis = [math.cos(angle) * u + math.sin(angle) * v for u, v in zip(x, y)]
        return {'status': 'estimated', 'axes_Base': [axis, cross(n, axis)],
                'table_normal_Base': n, 'yaw_table_deg': degrees, 'side_count': 2}

    def collect(self, geometry, xy=(.248, .004), start=None, orientation=None):
        events = list(geometry.plan_steps(xy, list(START if start is None else start), orientation))
        self.assertGreater(len(events), 1)
        self.assertEqual([i for i, event in enumerate(events) if event['stage'] == 'complete'],
                         [len(events) - 1])
        for event in events[:-1]:
            self.assertIsInstance(event['stage'], str)
            self.assertTrue(event['stage'])
            self.assertNotIn('plan', event, 'Only a complete four-stage result has plan authority')
        result = events[-1]['plan']
        self.assertTrue(all(stage in result for stage in STAGES))
        return events, result

    def assert_goals(self, plan, expected):
        self.assertEqual({stage: plan[stage] for stage in STAGES}, expected)

    def test_fixed_wrist_goals_match_pre_cooperative_center_and_left_fixtures(self):
        for xy_mm, expected in (((248., 4.), FROZEN_HELD), ((240., -20.), FROZEN_LEFT)):
            with self.subTest(xy_mm=xy_mm):
                geometry = self.geometry(xy_mm)
                xy = [v / 1000 for v in xy_mm]
                events, result = self.collect(geometry, xy)
                self.assert_goals(result, expected)
                self.assertEqual(result, self.geometry(xy_mm).plan(xy, START))
                self.assertGreaterEqual(len(events), 5)
                self.assertNotIn('wrist_alignment', result)

    def test_valid_wrist_orientation_preserves_frozen_aligned_goals(self):
        orientation = self.orientation(0)
        events, result = self.collect(self.geometry(enabled=True), orientation=orientation)
        self.assert_goals(result, FROZEN_ALIGNED)
        self.assertEqual(result['wrist_alignment']['status'], 'aligned')
        self.assertEqual(result, self.geometry(enabled=True).plan([.248, .004], START, orientation))
        self.assertGreaterEqual(len(events), 5)

    def test_valid_but_unreachable_alignment_keeps_exact_held_fallback(self):
        orientation = self.orientation(30)
        events, result = self.collect(self.geometry(enabled=True), orientation=orientation)
        self.assertEqual(result['wrist_alignment']['status'], 'held_current_fallback')
        self.assertGreater(result['wrist_alignment']['counts']['candidate_attempts'], 0)
        self.assert_goals(result, FROZEN_HELD)
        self.assertEqual(result, self.geometry(enabled=True).plan([.248, .004], START, orientation))
        self.assertGreater(len(events), 5)

    def test_missing_orientation_retains_initial_wrist_and_frozen_fallback(self):
        start = list(START)
        start[4] = 1707
        _, result = self.collect(self.geometry(enabled=True), start=start)
        self.assert_goals(result, FROZEN_MISSING_ORIENTATION)
        self.assertEqual(result['wrist_alignment']['status'], 'held_current_fallback')
        self.assertEqual(result, self.geometry(enabled=True).plan([.248, .004], start))

    def test_construction_is_inert_and_fixed_planning_yields_before_all_ik_finishes(self):
        geometry = self.geometry()
        with patch.object(geometry, 'solve', wraps=geometry.solve) as solve, \
                patch.object(geometry, 'report', wraps=geometry.report) as report:
            steps = geometry.plan_steps([.248, .004], START)
            solve.assert_not_called()
            report.assert_not_called()
            first = next(steps)
            self.assertNotEqual(first['stage'], 'complete')
            self.assertLess(solve.call_count, 3)
            steps.close()

    def test_fixed_stage_and_fk_report_order_is_unchanged(self):
        geometry = self.geometry()
        calls = []
        solve = geometry.solve
        report = geometry.report

        def solve_stage(*args):
            calls.append(('solve', args[1]))
            return solve(*args)

        def report_stage(counts):
            calls.append(('report', list(counts)))
            return report(counts)

        with patch.object(geometry, 'solve', side_effect=solve_stage), \
                patch.object(geometry, 'report', side_effect=report_stage):
            _, plan = self.collect(geometry)
        self.assertEqual([value for kind, value in calls if kind == 'solve'], [.08, .015, .08])
        self.assertEqual([value for kind, value in calls if kind == 'report'],
                         [plan[name] for name in STAGES])
        self.assertEqual([kind for kind, value in calls], ['solve'] * 3 + ['report'] * 4)

    def test_close_mid_fixed_search_does_not_continue_solving_or_build_execution(self):
        geometry = self.geometry()
        with patch.object(geometry, 'solve', wraps=geometry.solve) as solve, \
                patch.object(geometry, 'report', wraps=geometry.report) as report, \
                patch.object(pick_execution, 'open_pick_execution') as execution:
            steps = geometry.plan_steps([.248, .004], START)
            for _ in range(100):
                event = next(steps)
                self.assertNotEqual(event['stage'], 'complete')
                if solve.call_count:
                    break
            else:
                self.fail('Planner never advanced an IK stage')
            self.assertLess(solve.call_count, 3)
            before = solve.call_count, report.call_count
            steps.close()
            steps.close()
            with self.assertRaises(StopIteration):
                next(steps)
            self.assertEqual((solve.call_count, report.call_count), before)
            execution.assert_not_called()

    def test_wrist_planning_can_be_cancelled_between_progress_events(self):
        geometry = self.geometry(enabled=True)
        with patch.object(geometry, 'solve', wraps=geometry.solve) as solve, \
                patch.object(geometry, 'report', wraps=geometry.report) as report, \
                patch.object(pick_execution, 'open_pick_execution') as execution:
            steps = geometry.plan_steps([.248, .004], START, self.orientation(0))
            for _ in range(3):
                event = next(steps)
                self.assertNotEqual(event['stage'], 'complete')
            before = solve.call_count, report.call_count
            steps.close()
            with self.assertRaises(StopIteration):
                next(steps)
            self.assertEqual((solve.call_count, report.call_count), before)
            execution.assert_not_called()

    def test_all_planning_modes_use_no_uart_camera_or_target_sender(self):
        uart, pins, sensor = Mock(), Mock(), Mock()
        modules = {'machine': SimpleNamespace(UART=uart, FPIOA=pins),
                   'media.sensor': SimpleNamespace(Sensor=sensor)}
        with patch.dict(sys.modules, modules), \
                patch.object(pick_execution, 'open_pick_execution') as execution:
            self.collect(self.geometry())
            self.collect(self.geometry(enabled=True), orientation=self.orientation(0))
        uart.assert_not_called()
        pins.assert_not_called()
        sensor.assert_not_called()
        execution.assert_not_called()

    def test_exhausted_scene_still_raises_same_NoPickSolution_as_sync(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                geometry = self.geometry(enabled=enabled)
                with self.assertRaises(NoPickSolution) as sync:
                    geometry.plan([2., 0.], START)
                events = []
                with self.assertRaises(NoPickSolution) as cooperative:
                    for event in geometry.plan_steps([2., 0.], START):
                        events.append(event)
                self.assertIs(type(cooperative.exception), NoPickSolution)
                self.assertEqual(str(cooperative.exception), str(sync.exception))
                self.assertTrue(str(cooperative.exception).startswith('No complete four-stage IK:'))
                self.assertTrue(events)
                self.assertFalse(any(event['stage'] == 'complete' for event in events))

    def test_unexpected_solver_error_is_not_converted_to_no_solution(self):
        geometry = self.geometry()
        with patch.object(geometry, 'solve', side_effect=RuntimeError('solver unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'solver unavailable') as error:
                list(geometry.plan_steps([.248, .004], START))
        self.assertIs(type(error.exception), RuntimeError)

    def test_complete_is_the_only_terminal_event_and_no_more_work_follows(self):
        geometry = self.geometry()
        with patch.object(geometry, 'solve', wraps=geometry.solve) as solve:
            steps = geometry.plan_steps([.248, .004], START)
            events = list(steps)
            self.assertEqual(events[-1]['stage'], 'complete')
            before = solve.call_count
            with self.assertRaises(StopIteration):
                next(steps)
            self.assertEqual(solve.call_count, before)


if __name__ == '__main__':
    unittest.main()
