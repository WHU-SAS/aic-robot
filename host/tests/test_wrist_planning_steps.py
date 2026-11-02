"""Cooperative wrist planning keeps its math and cancels nested work promptly."""
import copy
import math
import unittest
from unittest.mock import patch

from device.k230.pick import wrist_plan


ORIENTATION = {'status': 'estimated', 'axes_Base': [[1., 0., 0.], [0., 1., 0.]],
               'table_normal_Base': [0., 0., 1.], 'yaw_table_deg': 0.}
IDENTITY = [[1., 0., 0., 0.], [0., 1., 0., 0.],
            [0., 0., 1., 0.], [0., 0., 0., 1.]]


class FakeGeometry:
    def __init__(self):
        self.trace = []
        self.open_angle = .5
        self.table_z_m = 0.
        self.pick = {'descend_tip_height_mm': 15, 'hover_tip_height_mm': 80,
                     'lift_tip_height_mm': 80, 'extra_closing_counts': 20,
                     'opening_count': 2332, 'cube_offset_toward_moving_finger_mm': 0}
        self.joints = [{'axis': [0., 0., 1.], 'software_count_limits': [0, 4095]}
                       for _ in range(6)]
        self.fail_elevated = set()

    def angle(self, axis, count):
        return (count-2048)*math.pi/2048

    def angles(self, counts):
        self.trace.append(('angles', tuple(counts)))
        return [self.angle(i+1, c) for i, c in enumerate(counts)]

    def metrics(self, q):
        self.trace.append(('metrics', tuple(q)))
        return [0., 0., 0.], {'wan2': [0., 0., 0.], 'zhua': [0., .02, 0.]}, \
            {'wan2': .015, 'zhua': .015}, {'wan2': IDENTITY}

    def solve(self, xy, height, tilt, seed, roll):
        self.trace.append(('solve', height, tilt, roll))
        if height == .08 and tilt in self.fail_elevated:
            raise ValueError('Rejected elevated tilt')
        counts = [2050 if height == .015 else 2100]*5+[2332]
        q = [0., 0., 0., math.radians(tilt), roll, self.open_angle]
        return counts, q

    def nominal_contact(self, q, orientation):
        self.trace.append(('contact', tuple(q)))
        return .25, .02

    def counts(self, q):
        self.trace.append(('counts', tuple(q)))
        return [2050]*5+[2200]

    def report(self, counts):
        self.trace.append(('report', tuple(counts)))
        return {'counts': list(counts)}


def fallback_plan():
    return {'hover': [2048]*6, 'descend': [2100]*6, 'close': [2100]*5+[2150],
            'lift': [2048]*5+[2150],
            'stage_tilts_deg': {'hover': 20, 'descend': 20, 'close': 20, 'lift': 20}}


class WristPlanningStepsTests(unittest.TestCase):
    def assert_final_only(self, events):
        completed = [e for e in events if e['stage'] == 'complete']
        self.assertEqual(len(completed), 1)
        self.assertIs(completed[0], events[-1])
        for event in events[:-1]:self.assertNotIn('result', event)
        return completed[0]['result']

    def test_root_search_keeps_sign_axis_order_counts_and_sync_results(self):
        a, b = FakeGeometry(), FakeGeometry()
        ca, cb = {'scalar_evaluations': 0}, {'scalar_evaluations': 0}
        events = list(wrist_plan.roll_candidates_steps(a, [.24, .01], 10, ORIENTATION, 0., ca))
        actual = self.assert_final_only(events)
        expected = wrist_plan.roll_candidates(b, [.24, .01], 10, ORIENTATION, 0., cb)
        self.assertEqual(actual, expected)
        self.assertEqual(ca, cb)
        self.assertEqual(a.trace, b.trace)
        self.assertEqual([(e['sign'], e['axis_index']) for e in events[:-1]],
                         [(1., 0), (1., 1), (-1., 0), (-1., 1)])
        self.assertLessEqual(len(actual), wrist_plan.MAX_ROOTS_PER_TILT)
        self.assertEqual(actual, sorted(actual, key=lambda r: abs(r[0])))

    def test_closing_root_generator_prevents_later_scalar_groups(self):
        g = FakeGeometry(); counters = {'scalar_evaluations': 0}
        steps = wrist_plan.roll_candidates_steps(g, [.24, .01], 10, ORIENTATION, 0., counters)
        self.assertEqual(g.trace, [])
        first = next(steps)
        self.assertEqual((first['sign'], first['axis_index']), (1., 0))
        before = (copy.deepcopy(counters), list(g.trace))
        steps.close()
        self.assertEqual(list(steps), [])
        self.assertEqual((counters, g.trace), before)

    def test_elevated_stage_yields_each_failed_tilt_before_trying_next(self):
        g = FakeGeometry(); g.fail_elevated = {5, 10}; counters = {'ik_calls': 0}
        steps = wrist_plan._elevated_stage_steps(g, [.24, .01], 80, 7.5, [0.]*6, 0., counters)
        for count, tilt in enumerate((5, 10, 0), 1):
            event = next(steps)
            self.assertEqual(event['stage'], 'wrist_elevated_solve')
            self.assertEqual(event['tilt_deg'], tilt)
            self.assertEqual(event['accepted'], tilt == 0)
            self.assertEqual(counters['ik_calls'], count)
        final = next(steps)
        self.assertEqual(final['stage'], 'complete')
        self.assertEqual(final['result'][2], 0)
        self.assertEqual(list(steps), [])
        other = FakeGeometry(); other.fail_elevated = {5, 10}; counts = {'ik_calls': 0}
        self.assertEqual(wrist_plan._elevated_stage(other, [.24, .01], 80, 7.5, [0.]*6, 0., counts),
                         final['result'])
        self.assertEqual(other.trace, g.trace)
        self.assertEqual(counts, counters)

    def test_elevated_cancel_does_not_try_another_tilt(self):
        g = FakeGeometry(); g.fail_elevated = {5}; counters = {'ik_calls': 0}
        steps = wrist_plan._elevated_stage_steps(g, [.24, .01], 80, 7.5, [0.]*6, 0., counters)
        self.assertFalse(next(steps)['accepted'])
        before = list(g.trace)
        steps.close()
        self.assertEqual(g.trace, before)
        self.assertEqual(counters['ik_calls'], 1)

    def test_stage_generator_matches_sync_math_and_reports_in_original_order(self):
        a, b = FakeGeometry(), FakeGeometry()
        ca, cb = {'ik_calls': 0}, {'ik_calls': 0}
        events = list(wrist_plan._stages_steps(a, [.24, .01], [2048]*6, 10, 0., ORIENTATION, ca))
        actual = self.assert_final_only(events)
        expected = wrist_plan._stages(b, [.24, .01], [2048]*6, 10, 0., ORIENTATION, cb)
        self.assertEqual(actual, expected)
        self.assertEqual(a.trace, b.trace)
        self.assertEqual(ca, cb)
        self.assertEqual(ca['ik_calls'], 3)
        self.assertEqual(actual['close'][5], 2180)
        self.assertEqual(actual['lift'][5], 2180)
        self.assertEqual([e['name'] for e in events if e['stage'] == 'wrist_fk_report'],
                         ['hover', 'descend', 'close', 'lift'])
        self.assertEqual(list(actual['target_FK']), ['hover', 'descend', 'close', 'lift'])

    def test_stage_cancel_at_low_closing_elevated_or_report_boundary_stops_work(self):
        for boundary in ('wrist_low_solve', 'wrist_closing_geometry',
                         'wrist_elevated_solve', 'wrist_fk_report'):
            with self.subTest(boundary=boundary):
                g = FakeGeometry(); counters = {'ik_calls': 0}
                steps = wrist_plan._stages_steps(g, [.24, .01], [2048]*6, 10, 0., ORIENTATION, counters)
                for event in steps:
                    self.assertNotEqual(event['stage'], 'complete')
                    if event['stage'] == boundary:break
                before = (list(g.trace), dict(counters))
                steps.close()
                self.assertEqual(list(steps), [])
                self.assertEqual((g.trace, counters), before)

    def test_plan_exhausts_root_grid_then_prefers_smaller_wrist_pair(self):
        g = FakeGeometry(); trace = []; closed = []
        def roots(g, xy, tilt, orientation, initial, counters):
            trace.append(('roots', tilt))
            counters['scalar_evaluations'] += 3
            yield {'stage': 'fake_roots', 'tilt_deg': tilt}
            yield {'stage': 'complete', 'result':
                   [(.8, 0, 1.), (.1, 1, 1.)] if tilt == 0 else [(.02, 1, 1.)]}
        def stages(g, xy, start, tilt, roll, orientation, counters):
            trace.append(('candidate', tilt, roll)); counters['ik_calls'] += 1
            try:
                yield {'stage': 'fake_candidate'}
                if tilt == 0:raise ValueError('First near-pair tilt rejected')
                yield {'stage': 'complete', 'result': fallback_plan()}
            finally:closed.append(tilt)
        def fallback(*args):
            self.fail('Aligned success must not run fallback')
        with patch.object(wrist_plan, 'TILTS', (0, 5)), \
                patch.object(wrist_plan, 'roll_candidates_steps', side_effect=roots), \
                patch.object(wrist_plan, '_stages_steps', side_effect=stages), \
                patch.object(wrist_plan, 'jaw_direction', return_value=[1., 0., 0.]), \
                patch.object(wrist_plan, 'direction_error', return_value=0.):
            events = list(wrist_plan.plan_steps(g, [.24, .01], [2048]*6, ORIENTATION, fallback))
        result = self.assert_final_only(events)
        self.assertEqual(trace, [('roots', 0), ('roots', 5), ('candidate', 0, .1), ('candidate', 5, .02)])
        self.assertEqual(closed, [0, 5])
        alignment = result['wrist_alignment']
        self.assertEqual(alignment['axis_preference'], [1, 0])
        self.assertEqual(alignment['cube_axis_index'], 1)
        self.assertEqual(alignment['counts'], {'scalar_evaluations': 6, 'candidate_attempts': 2, 'ik_calls': 2})
        self.assertEqual(alignment['failures'], {'First near-pair tilt rejected': 1})
        self.assertEqual(sum(e['stage'] == 'wrist_candidate_rejected' for e in events), 1)

    def test_plan_cancel_closes_active_root_child_without_trying_next_tilt(self):
        closed = []; calls = []
        def roots(*args):
            calls.append('root')
            try:
                yield {'stage': 'fake_root_progress'}
                self.fail('Cancelled root work resumed')
            finally:closed.append('root')
        with patch.object(wrist_plan, 'roll_candidates_steps', side_effect=roots):
            steps = wrist_plan.plan_steps(FakeGeometry(), [.24, .01], [2048]*6, ORIENTATION, None)
            self.assertEqual(next(steps)['stage'], 'fake_root_progress')
            steps.close()
        self.assertEqual(calls, ['root'])
        self.assertEqual(closed, ['root'])

    def test_plan_cancel_closes_candidate_child_before_another_candidate(self):
        calls = []; closed = []
        def roots(*args):
            yield {'stage': 'complete', 'result': [(.1, 0, 1.), (.2, 0, 1.)]}
        def stages(*args):
            calls.append('candidate')
            try:
                yield {'stage': 'fake_stage_progress'}
                self.fail('Cancelled candidate resumed')
            finally:closed.append('candidate')
        with patch.object(wrist_plan, 'TILTS', (0,)), \
                patch.object(wrist_plan, 'roll_candidates_steps', side_effect=roots), \
                patch.object(wrist_plan, '_stages_steps', side_effect=stages):
            steps = wrist_plan.plan_steps(FakeGeometry(), [.24, .01], [2048]*6, ORIENTATION, None)
            self.assertEqual(next(steps)['stage'], 'wrist_candidate')
            self.assertEqual(next(steps)['stage'], 'fake_stage_progress')
            steps.close()
        self.assertEqual(calls, ['candidate'])
        self.assertEqual(closed, ['candidate'])

    def test_invalid_orientation_keeps_exact_sync_fallback_and_closes_child(self):
        g = FakeGeometry(); closed = []; calls = []
        def fallback_steps(xy, start):
            calls.append((xy, start))
            try:
                yield {'stage': 'held_report'}
                yield {'stage': 'complete', 'result': fallback_plan()}
            finally:closed.append(True)
        events = list(wrist_plan.plan_steps(g, [.24, .01], [2048]*6, None, fallback_steps))
        result = self.assert_final_only(events)
        self.assertEqual(len(calls), 1)
        self.assertEqual(closed, [True])
        self.assertEqual([e['stage'] for e in events], ['wrist_fallback', 'held_report', 'complete'])
        expected = wrist_plan.plan(FakeGeometry(), [.24, .01], [2048]*6, None,
                                   lambda xy, start: fallback_plan())
        self.assertEqual(result, expected)
        self.assertEqual(result['wrist_alignment']['status'], 'held_current_fallback')
        self.assertEqual(result['wrist_alignment']['failures'], {'No verified side orientation': 1})

    def test_cancel_fallback_closes_it_and_never_publishes_plan(self):
        closed = []
        def fallback(*args):
            try:
                yield {'stage': 'held_solve'}
                self.fail('Cancelled held fallback resumed')
            finally:closed.append(True)
        steps = wrist_plan.plan_steps(FakeGeometry(), [.24, .01], [2048]*6, None, fallback)
        self.assertEqual(next(steps)['stage'], 'wrist_fallback')
        self.assertEqual(next(steps)['stage'], 'held_solve')
        steps.close()
        self.assertEqual(closed, [True])
        self.assertEqual(list(steps), [])

    def test_fallback_no_solution_type_and_resource_errors_are_not_generalized(self):
        class SpecificNoPickSolution(ValueError):pass
        for error in (SpecificNoPickSolution('No held pose'), MemoryError('Planning buffer unavailable')):
            with self.subTest(error=type(error).__name__):
                def fallback(*args):
                    yield {'stage': 'held_solve'}
                    raise error
                steps = wrist_plan.plan_steps(FakeGeometry(), [.24, .01], [2048]*6, None, fallback)
                with self.assertRaises(type(error)) as caught:list(steps)
                self.assertIs(caught.exception, error)
                def synchronous(*args):raise error
                with self.assertRaises(type(error)) as caught:
                    wrist_plan.plan(FakeGeometry(), [.24, .01], [2048]*6, None, synchronous)
                self.assertIs(caught.exception, error)

    def test_real_45_degree_scene_preserves_frozen_goals_and_search_counters(self):
        from tools.deploy.k230_pick import build_bundle
        from device.k230.pick.kinematics import Geometry
        from device.k230.pick.cube_orientation import plane_basis
        from device.k230.pick.numeric import cross
        config, unused, unused_manifest = build_bundle()
        plane = config['vision']['localization']['table_plane_Base_mm']
        x, y, normal = plane_basis(plane)
        angle = math.radians(45)
        axis = [math.cos(angle)*u+math.sin(angle)*v for u, v in zip(x, y)]
        orientation = {'status': 'estimated', 'axes_Base': [axis, cross(normal, axis)],
                       'table_normal_Base': normal, 'yaw_table_deg': 45}
        xy = [.248, .004]
        z = (plane['offset_mm']-normal[0]*248-normal[1]*4)/normal[2]
        g = Geometry(config['model'], dict(config['pick'], cube_offset_toward_moving_finger_mm=0), z)
        def fallback(xy, start):
            yield {'stage': 'complete', 'result': g._plan_held(xy, start)}
        result = self.assert_final_only(list(wrist_plan.plan_steps(g, xy, [2048]*6, orientation, fallback)))
        self.assertEqual({stage: result[stage] for stage in ('hover', 'descend', 'close', 'lift')}, {
            'hover': [2059, 2624, 2505, 2893, 1543, 2332],
            'descend': [2059, 2905, 2317, 2884, 1543, 2332],
            'close': [2059, 2905, 2317, 2884, 1543, 2158],
            'lift': [2059, 2624, 2505, 2893, 1543, 2158]})
        self.assertEqual(result['wrist_alignment']['counts'],
                         {'scalar_evaluations': 407, 'candidate_attempts': 3, 'ik_calls': 11})


if __name__ == '__main__':unittest.main()
