"""Cooperative arm execution with fake time/IO; never opens a real UART."""
import copy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from device.k230.arm.pick_execution import PickExecution


POLICY = {
    'hover_tolerance_counts': 30,
    'descend_tolerance_counts': 10,
    'descend_axis4_lead_ms': 200,
    'gripper_stable_samples': 5,
    'gripper_stable_tolerance_counts': 1,
    'poll_interval_ms': 50,
    'grip_confirmation_delta_counts': 30,
    'lift_tolerance_counts': 10,
    'lift_gripper_stable_tolerance_counts': 1,
    'lift_gripper_stable_ms': 200,
    'recovery_observation_delay_ms': 500,
}
PLAN = {
    'hover': [2000, 2001, 2002, 2003, 2004, 2332],
    'descend': [2100, 2101, 2102, 2103, 2104, 2332],
    'close': [2100, 2101, 2102, 2103, 2104, 2150],
    'lift': [2200, 2201, 2202, 2203, 2204, 2150],
}
ALL_IDS = [1, 2, 3, 4, 5, 6]


class Clock:
    def __init__(self, start=0, period=None):
        self.elapsed = start
        self.period = period

    def now(self):
        return self.elapsed if self.period is None else self.elapsed % self.period

    def advance(self, ms):
        self.elapsed += ms

    def diff(self, a, b):
        if self.period is None:
            return a - b
        return (a - b + self.period // 2) % self.period - self.period // 2


class FakeIO:
    """Only exposes user-authorized goal/read/ready/release operations."""
    def __init__(self, clock):
        self.clock = clock
        self.positions = list(PLAN['hover'])
        self.events = []
        self.fail_read = False
        self.fail_send = False
        self.fail_ready = False
        self.read_cost_ms = 0

    def send_goals(self, goals, servo_ids):
        self.events.append(('goals', self.clock.now(), list(goals), list(servo_ids)))
        if self.fail_send:
            raise RuntimeError('fake short goal write')

    def read_positions(self, servo_ids):
        self.events.append(('read', self.clock.now(), list(servo_ids)))
        self.clock.advance(self.read_cost_ms)
        if self.fail_read:
            self.fail_read = False
            raise RuntimeError('fake missing feedback')
        return [self.positions[i - 1] for i in servo_ids]

    def send_ready2(self):
        self.events.append(('ready2', self.clock.now()))
        if self.fail_ready:
            raise RuntimeError('fake short ready write')
        return {'goal_sent_ticks_ms': self.clock.now()}

    def close(self):
        self.events.append(('close', self.clock.now()))


class PickExecutionTests(unittest.TestCase):
    def make(self, policy=None, clock=None):
        self.clock = clock or Clock()
        self.io = FakeIO(self.clock)
        self.plan = copy.deepcopy(PLAN)
        self.policy = dict(POLICY)
        self.policy.update(policy or {})
        self.emitted = []
        self.execution = PickExecution(self.plan, self.policy, self.io, 7,
                                       self.clock.now, self.clock.diff,
                                       emit=lambda name, details: self.emitted.append((name, details)))
        return self.execution

    def step(self, after=0, version=7, allow_success=None):
        self.clock.advance(after)
        kwargs = {'target_version': version}
        if allow_success is not None:
            kwargs['allow_success'] = allow_success
        return self.execution.step(**kwargs)

    def observe(self, duration, allow_success=None):
        """Advance real sampling turns, not just two distant clock endpoints."""
        state = self.execution.snapshot()
        while duration:
            interval = min(50, duration)
            state = self.step(interval, allow_success=allow_success)
            duration -= interval
        return state

    def goals(self):
        return [e for e in self.io.events if e[0] == 'goals']

    def ready(self):
        return [e for e in self.io.events if e[0] == 'ready2']

    def reads(self):
        return [e for e in self.io.events if e[0] == 'read']

    def begin_close(self):
        self.step()                     # hover goal
        self.step(50)                   # six-axis hover gate
        self.step()                     # descend axis 4
        self.step(200)                  # remaining five goals
        self.io.positions = list(self.plan['descend'])
        self.step(50)                   # six-axis descend gate
        self.step()                     # close goal
        self.assertEqual(self.goals()[-1][2], self.plan['close'])

    def begin_lift(self, delta=50):
        self.begin_close()
        self.io.positions[5] = self.plan['close'][5] + delta
        for _ in range(5):
            self.step(50)
        if delta >= 30:
            self.step()                 # lift goal, on a separate step
            self.assertEqual(self.goals()[-1][2], self.plan['lift'])
            self.io.positions[:5] = self.plan['lift'][:5]

    def test_construct_and_idle_snapshot_do_not_issue_io(self):
        execution = self.make()
        self.assertEqual(self.io.events, [])
        state = execution.snapshot()
        self.assertEqual(state['status'], 'running')
        self.assertEqual(state['target_version'], 7)
        self.assertFalse(state['can_reobserve'])
        self.assertEqual(self.io.events, [])

    def test_hover_gate_uses_same_round_six_axes_and_inclusive_30(self):
        self.make()
        self.step()
        self.assertEqual(self.goals(), [('goals', 0, self.plan['hover'], ALL_IDS)])
        self.io.positions[0] += 31
        self.step(50)
        self.io.positions = list(self.plan['hover'])
        self.io.positions[1] += 31
        self.step(50)
        self.assertEqual(len(self.goals()), 1)
        self.io.positions = [v + (-30 if i % 2 else 30)
                             for i, v in enumerate(self.plan['hover'])]
        self.step(50)
        self.assertEqual(len(self.goals()), 1)
        self.step()
        self.assertEqual(self.goals()[-1][3], [4])
        self.assertTrue(all(e[2] == ALL_IDS for e in self.reads()))

    def test_fourth_axis_lead_is_200ms_and_each_step_yields(self):
        self.make()
        self.step()
        self.step(50)
        self.step()
        first = self.goals()[-1]
        self.assertEqual(first[3], [4])
        self.step(199)
        self.assertEqual(len(self.goals()), 2)
        self.step(1)
        second = self.goals()[-1]
        self.assertEqual(second[3], [1, 2, 3, 5, 6])
        self.assertEqual(self.clock.diff(second[1], first[1]), 200)
        self.assertEqual(first[2], self.plan['descend'])
        self.assertEqual(second[2], self.plan['descend'])

    def test_cancel_during_lead_replaces_target_and_never_sends_delayed_packet(self):
        self.make()
        self.step()
        self.step(50)
        self.step()
        self.step(100)
        self.execution.invalidate_target('target_moved')
        self.assertEqual(len(self.ready()), 0)
        state = self.step()
        self.assertEqual(state['status'], 'recovering')
        self.assertEqual(len(self.ready()), 1)
        for wait in (100, 400, 50):
            self.step(wait)
        self.assertEqual([e[3] for e in self.goals()], [ALL_IDS, [4]])
        self.assertEqual(len(self.ready()), 1)

    def test_descend_gate_uses_all_six_with_inclusive_10(self):
        self.make()
        self.step()
        self.step(50)
        self.step()
        self.step(200)
        self.io.positions = list(self.plan['descend'])
        self.io.positions[5] += 11
        self.step(50)
        self.assertEqual(len(self.goals()), 3)
        self.io.positions = [v + (-10 if i % 2 else 10)
                             for i, v in enumerate(self.plan['descend'])]
        self.step(50)
        self.step()
        self.assertEqual(self.goals()[-1][2], self.plan['close'])

    def test_empty_grip_waits_for_stable_window_then_recovers_without_lift(self):
        self.make()
        self.begin_close()
        self.io.positions[5] = self.plan['close'][5] + 29
        for _ in range(4):
            self.step(50)
            self.assertEqual(self.ready(), [])
        state = self.step(50)
        self.assertEqual(state['status'], 'recovering')
        self.assertIsNone(state['grasp_confirmed'])
        self.assertIs(state['grip_confirmation']['grasp_confirmed'], False)
        self.assertEqual(state['grip_confirmation']['delta_counts'], 29)
        self.assertEqual(state['grip_confirmation']['phase'], 'close')
        self.assertEqual(state['recovery_reason'], 'empty_grip')
        self.assertEqual(self.ready(), [('ready2', self.clock.now())])
        self.assertNotIn(self.plan['lift'], [e[2] for e in self.goals()])

    def test_close_uses_last_stable_sample_and_30_is_inclusive(self):
        for deltas, expected in (([31, 30, 31, 30, 29], False), ([30] * 5, True)):
            with self.subTest(deltas=deltas):
                self.make()
                self.begin_close()
                for delta in deltas:
                    self.io.positions[5] = self.plan['close'][5] + delta
                    state = self.step(50)
                self.assertIs(state['grasp_confirmed'], True if expected else None)
                self.assertIs(state['grip_confirmation']['grasp_confirmed'], expected)
                self.assertEqual(state['grip_confirmation']['delta_counts'], deltas[-1])
                self.assertEqual(len(self.ready()), 0 if expected else 1)

    def test_close_does_not_accept_single_count_creep(self):
        self.make()
        self.begin_close()
        for delta in (50, 51, 52, 53, 54):
            self.io.positions[5] = self.plan['close'][5] + delta
            self.step(50)
        self.assertEqual(self.goals()[-1][2], self.plan['close'])
        self.assertEqual(self.ready(), [])
        self.assertIsNot(self.execution.snapshot()['grasp_confirmed'], True)

    def test_lift_excludes_blocked_axis6_and_waits_200ms_after_arrival(self):
        self.make()
        self.begin_lift(delta=30)
        state = self.step(50)
        self.assertEqual(state['status'], 'running')
        self.observe(150)
        self.assertNotEqual(self.step(49)['status'], 'succeeded')
        state = self.step(1)
        self.assertEqual(state['status'], 'succeeded')
        self.assertIs(state['grasp_confirmed'], True)
        self.assertEqual(state['grip_confirmation']['delta_counts'], 30)
        self.assertEqual(self.ready(), [])

    def test_lift_has_own_10_count_gate_and_cannot_accumulate_time_before_arrival(self):
        self.make()
        self.begin_lift()
        self.io.positions[1] += 11
        for _ in range(6):
            self.assertNotEqual(self.step(50)['status'], 'succeeded')
        self.io.positions[1] -= 1  # exactly +10 is accepted
        self.step(50)
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_gate_is_same_round_not_latched_per_axis(self):
        self.make()
        self.begin_lift()
        self.io.positions[0] += 11
        self.step(50)
        self.io.positions[0] -= 11
        self.io.positions[1] += 11
        for _ in range(5):
            self.assertNotEqual(self.step(50)['status'], 'succeeded')
        self.io.positions[1] -= 11
        self.step(50)
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_out_of_band_keeps_grip_and_restarts_fixed_band_timer(self):
        self.make()
        self.begin_lift()
        self.step(50)
        self.observe(150)
        self.io.positions[5] += 2
        self.assertEqual(self.step(50)['status'], 'running')
        self.assertEqual(self.ready(), [])
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_drift_never_passes_merely_because_adjacent_changes_are_one(self):
        self.make()
        self.begin_lift()
        for _ in range(12):
            self.assertNotEqual(self.step(50)['status'], 'succeeded')
            self.io.positions[5] += 1
        self.assertEqual(self.ready(), [])
        for _ in range(6):
            state = self.step(50)
        self.assertEqual(state['status'], 'succeeded')

    def test_lift_stability_tolerance_is_separate_from_close_tolerance(self):
        self.make({'lift_gripper_stable_tolerance_counts': 2})
        self.begin_close()
        for delta in (50, 52, 50, 52, 54):
            self.io.positions[5] = self.plan['close'][5] + delta
            self.step(50)
        self.assertEqual(self.goals()[-1][2], self.plan['close'])
        for _ in range(5):
            self.step(50)
        self.step()
        self.io.positions[:5] = self.plan['lift'][:5]
        self.step(50)
        self.io.positions[5] += 2
        self.step(50)
        self.observe(100)
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_losing_arrival_resets_success_interval(self):
        self.make()
        self.begin_lift()
        self.step(50)
        self.observe(150)
        self.io.positions[3] += 11
        self.assertNotEqual(self.step(50)['status'], 'succeeded')
        self.io.positions[3] -= 11
        self.step(50)
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_one_second_observation_gap_does_not_count_as_stability(self):
        self.make()
        self.begin_lift()
        self.step(50)  # first arrived sample starts observation
        state = self.step(1000)
        self.assertEqual(state['status'], 'running')
        self.assertIs(state['grasp_confirmed'], True)
        self.assertEqual(self.ready(), [])
        self.assertIn('lift_observation_gap', [name for name, details in self.emitted])
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_observation_gap_limit_is_twice_existing_poll_period(self):
        for gap, resets in ((100, False), (101, True)):
            with self.subTest(gap=gap):
                self.make()
                self.begin_lift()
                self.step(50)
                self.assertEqual(self.step(gap)['status'], 'running')
                if resets:
                    self.assertNotEqual(self.observe(150)['status'], 'succeeded')
                    state = self.step(50)
                else:
                    state = self.observe(100)
                self.assertEqual(state['status'], 'succeeded')
                self.assertEqual('lift_observation_gap' in [e[0] for e in self.emitted], resets)

    def test_gap_restarts_fixed_band_and_later_jitter_restarts_it_again(self):
        self.make()
        self.begin_lift()
        self.step(50)
        self.io.positions[5] += 2
        self.assertEqual(self.step(1000)['status'], 'running')
        self.io.positions[5] += 1
        self.assertEqual(self.step(50)['status'], 'running')
        self.io.positions[5] += 1  # adjacent +1, but outside fixed reference +2
        self.assertEqual(self.step(50)['status'], 'running')
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        result = self.step(50)
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(result['grip_confirmation']['stable_reference'], self.io.positions[5])
        self.assertEqual(self.ready(), [])

    def test_observation_gap_crossing_ticks_wrap_still_restarts_stability(self):
        self.make(clock=Clock(start=1400, period=2048))
        self.begin_lift()
        self.step(50)
        before = self.clock.now()
        state = self.step(1000)
        self.assertLess(self.clock.now(), before)
        self.assertEqual(state['status'], 'running')
        self.assertIn('lift_observation_gap', [name for name, details in self.emitted])
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_realistic_180ms_read_plus_50ms_idle_can_confirm_stability(self):
        self.make()
        self.begin_lift()
        self.io.read_cost_ms = 180
        first = self.step(50)
        self.assertEqual(first['status'], 'running')
        self.assertEqual(first['last_sample']['read_ms'], 180)
        first_end = first['last_sample']['timestamp_ms']
        result = self.step(50)
        self.assertEqual(result['status'], 'succeeded')
        sample = result['last_sample']
        self.assertEqual(self.clock.diff(sample['timestamp_ms'], first_end), 230)
        self.assertEqual(self.clock.diff(sample['read_started_ms'], first_end), 50)
        self.assertEqual(sample['read_ms'], 180)
        self.assertEqual(result['grip_confirmation']['stable_ms'], 230)
        self.assertNotIn('lift_observation_gap', [name for name, details in self.emitted])

    def test_extra_idle_gap_still_restarts_window_with_slow_reads(self):
        self.make()
        self.begin_lift()
        self.io.read_cost_ms = 180
        self.step(50)
        state = self.step(1000)
        self.assertEqual(state['status'], 'running')
        gaps = [details for name, details in self.emitted if name == 'lift_observation_gap']
        self.assertEqual(len(gaps), 1)
        self.assertEqual(gaps[0]['idle_gap_ms'], 1000)
        self.assertEqual(gaps[0]['sample_gap_ms'], 1180)
        self.assertEqual(gaps[0]['read_ms'], 180)
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_idle_100_101_boundary_excludes_the_180ms_read_duration(self):
        for idle, resets in ((100, False), (101, True)):
            with self.subTest(idle=idle):
                self.make()
                self.begin_lift()
                self.io.read_cost_ms = 180
                first = self.step(50)
                first_end = first['last_sample']['timestamp_ms']
                state = self.step(idle)
                self.assertEqual(self.clock.diff(state['last_sample']['timestamp_ms'], first_end), idle + 180)
                gaps = [details for name, details in self.emitted if name == 'lift_observation_gap']
                if resets:
                    self.assertEqual(state['status'], 'running')
                    self.assertEqual(len(gaps), 1)
                    self.assertEqual(gaps[0]['idle_gap_ms'], 101)
                    self.assertEqual(gaps[0]['sample_gap_ms'], 281)
                    self.assertEqual(self.step(50)['status'], 'succeeded')
                else:
                    self.assertEqual(state['status'], 'succeeded')
                    self.assertEqual(gaps, [])

    def test_real_band_exit_still_restarts_stability_with_slow_reads(self):
        self.make()
        self.begin_lift()
        self.io.read_cost_ms = 180
        self.step(50)
        self.io.positions[5] += 2
        state = self.step(50)
        self.assertEqual(state['status'], 'running')
        self.assertIs(state['grasp_confirmed'], True)
        self.assertEqual(self.ready(), [])
        result = self.step(50)
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(result['grip_confirmation']['stable_reference'], self.io.positions[5])
        self.assertEqual(result['grip_confirmation']['stable_ms'], 230)

    def test_failed_slow_read_is_unknown_and_requires_a_new_stability_window(self):
        self.make()
        self.begin_lift()
        self.io.read_cost_ms = 180
        self.step(50)
        self.io.fail_read = True
        unknown = self.step(50)
        self.assertEqual(unknown['status'], 'running')
        self.assertEqual(unknown['feedback_status'], 'unknown')
        self.assertIsNone(unknown['grasp_confirmed'])
        self.assertFalse(unknown['success_ready'])
        self.assertEqual(self.ready(), [])
        self.assertEqual(self.step(50)['status'], 'running')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_slow_read_timing_across_tick_wrap_preserves_only_50ms_idle(self):
        self.make(clock=Clock(start=1100, period=2048))
        self.begin_lift()
        self.io.read_cost_ms = 180
        first = self.step(50)
        first_end = first['last_sample']['timestamp_ms']
        result = self.step(50)
        self.assertLess(result['last_sample']['timestamp_ms'], first_end)
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(result['last_sample']['read_ms'], 180)
        self.assertEqual(self.clock.diff(result['last_sample']['read_started_ms'], first_end), 50)
        self.assertEqual(result['grip_confirmation']['stable_ms'], 230)
        self.assertNotIn('lift_observation_gap', [name for name, details in self.emitted])

    def test_lift_delta29_recovers_on_first_valid_sample(self):
        self.make()
        self.begin_lift()
        self.io.positions[5] = self.plan['close'][5] + 29
        state = self.step(50)
        self.assertEqual(state['status'], 'recovering')
        self.assertIsNone(state['grasp_confirmed'])
        self.assertIs(state['grip_confirmation']['grasp_confirmed'], False)
        self.assertEqual(state['grip_confirmation']['phase'], 'lift')
        self.assertEqual(state['recovery_reason'], 'grip_lost')
        self.assertEqual(len(self.ready()), 1)

    def test_close_read_failure_clears_partial_stability(self):
        self.make()
        self.begin_close()
        self.io.positions[5] = self.plan['close'][5] + 50
        for _ in range(4):
            self.step(50)
        self.io.fail_read = True
        unknown = self.step(50)
        self.assertNotEqual(unknown['status'], 'succeeded')
        self.assertEqual(unknown['feedback_status'], 'unknown')
        self.assertIsNone(unknown['grasp_confirmed'])
        self.assertEqual(self.ready(), [])
        for _ in range(4):
            self.step(50)
            self.assertEqual(self.goals()[-1][2], self.plan['close'])
        self.step(50)
        self.assertIs(self.execution.snapshot()['grasp_confirmed'], True)

    def test_lift_read_failure_is_unknown_not_empty_and_restarts_confirmation(self):
        self.make()
        self.begin_lift()
        self.step(50)
        self.observe(150)
        self.io.fail_read = True
        unknown = self.step(50)
        self.assertNotEqual(unknown['status'], 'succeeded')
        self.assertEqual(unknown['feedback_status'], 'unknown')
        self.assertIsNone(unknown['grasp_confirmed'])
        self.assertEqual(self.ready(), [])
        self.step(50)
        self.assertNotEqual(self.observe(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_recovery_reobserves_at_500ms_without_any_arrival_read(self):
        self.make()
        self.execution.invalidate_target()
        state = self.step()
        sent_at = state['recovery_goal_sent_ms']
        self.assertFalse(state['can_reobserve'])
        self.assertEqual(self.reads(), [])
        self.assertFalse(self.step(499)['can_reobserve'])
        state = self.step(1)
        self.assertEqual(state['status'], 'reobserve_ready')
        self.assertTrue(state['can_reobserve'])
        self.assertEqual(state['recovery_goal_sent_ms'], sent_at)
        self.assertEqual(self.reads(), [])
        self.assertEqual(self.goals(), [])
        self.assertEqual(len(self.ready()), 1)

    def test_changed_target_version_recovers_instead_of_rebinding_old_plan(self):
        self.make()
        self.step()
        state = self.step(50, version=8)
        self.assertEqual(state['status'], 'recovering')
        self.assertEqual(state['target_version'], 7)
        self.assertEqual(self.reads(), [])
        self.step(500, version=9)
        for _ in range(4):
            self.step(50, version=7)
        self.assertEqual(len(self.ready()), 1)
        self.assertEqual(len(self.goals()), 1)

    def test_repeated_invalidation_cannot_restart_recovery_or_old_goals(self):
        self.make()
        self.step()
        self.execution.invalidate_target()
        self.step()
        for _ in range(5):
            self.execution.invalidate_target('target_moved_again')
            state = self.step(100)
        self.assertTrue(state['can_reobserve'])
        self.assertEqual(len(self.ready()), 1)
        self.assertEqual(len(self.goals()), 1)

    def test_invalidation_preempts_pending_close_and_pending_lift(self):
        for pending in ('close', 'lift'):
            with self.subTest(pending=pending):
                self.make()
                if pending == 'close':
                    self.step()
                    self.step(50)
                    self.step()
                    self.step(200)
                    self.io.positions = list(self.plan['descend'])
                    self.step(50)
                else:
                    self.begin_close()
                    self.io.positions[5] = self.plan['close'][5] + 50
                    for _ in range(5):
                        self.step(50)
                self.assertEqual(self.execution.snapshot()['pending_stage'], pending)
                before = len(self.goals())
                self.execution.invalidate_target()
                self.step()
                self.step(500)
                self.assertEqual(len(self.goals()), before)
                self.assertEqual(len(self.ready()), 1)

    def test_ready_opening_makes_previous_positive_grip_historical_only(self):
        self.make()
        self.begin_close()
        self.io.positions[5] = self.plan['close'][5] + 50
        for _ in range(5):
            confirmed = self.step(50)
        self.assertIs(confirmed['grasp_confirmed'], True)
        historical = dict(confirmed['grip_confirmation'])
        self.execution.invalidate_target()
        for delay, expected_status in ((0, 'recovering'), (500, 'reobserve_ready')):
            state = self.step(delay)
            self.assertEqual(state['status'], expected_status)
            self.assertIsNone(state['grasp_confirmed'])
            self.assertEqual(state['feedback_status'], 'unknown')
            self.assertFalse(state['target_valid'])
            self.assertEqual(state['grip_confirmation'], historical)
        self.assertEqual(historical['phase'], 'close')
        self.assertEqual(historical['sample_time_ms'], confirmed['last_sample']['timestamp_ms'])
        self.assertEqual(len(self.ready()), 1)
        self.assertNotIn(self.plan['lift'], [e[2] for e in self.goals()])

    def test_constructor_copies_plan_goals_so_owner_cannot_rewrite_pending_action(self):
        self.make()
        self.plan['hover'][0] += 100
        self.step()
        self.assertEqual(self.goals()[0][2], PLAN['hover'])

    def test_goal_send_fault_is_latched_without_retry_or_recovery(self):
        self.make()
        self.io.fail_send = True
        state = self.step()
        self.assertEqual(state['status'], 'fault')
        for _ in range(3):
            self.step(500, version=8)
        self.assertEqual(len(self.goals()), 1)
        self.assertEqual(self.ready(), [])

    def test_recovery_send_fault_is_latched_without_automatic_retry(self):
        self.make()
        self.io.fail_ready = True
        self.execution.invalidate_target()
        self.assertEqual(self.step()['status'], 'fault')
        self.step(500)
        self.execution.invalidate_target()
        self.step()
        self.assertEqual(len(self.ready()), 1)
        self.assertEqual(self.goals(), [])

    def test_close_releases_resources_once_and_never_sends_recovery_or_future_goals(self):
        self.make()
        self.step()
        before = len(self.io.events)
        self.execution.close()
        self.execution.close()
        self.execution.invalidate_target()
        self.assertEqual(self.step(500, version=8)['status'], 'closed')
        self.assertEqual(self.io.events[before:], [('close', self.clock.now() - 500)])

    def test_success_is_terminal_and_step_does_not_repeat_writes_or_reads(self):
        self.make()
        self.begin_lift()
        self.step(50)
        self.observe(200)
        self.assertEqual(self.execution.snapshot()['status'], 'succeeded')
        before = list(self.io.events)
        self.step(500)
        self.assertEqual(self.io.events, before)

    def test_success_can_be_deferred_without_stopping_fresh_feedback(self):
        self.make()
        self.begin_lift()
        deferred = self.observe(250, allow_success=False)
        self.assertEqual(deferred['status'], 'running')
        self.assertEqual(deferred['phase'], 'lift')
        self.assertTrue(deferred['success_ready'])
        self.assertIs(deferred['grasp_confirmed'], True)
        before = len(self.reads())
        self.assertTrue(self.step(50, allow_success=False)['success_ready'])
        self.assertEqual(len(self.reads()), before + 1)
        # Merely allowing success cannot commit stale feedback between polls.
        self.assertEqual(self.step(allow_success=True)['status'], 'running')
        self.assertEqual(len(self.reads()), before + 1)
        accepted = self.step(50, allow_success=True)
        self.assertEqual(accepted['status'], 'succeeded')
        self.assertEqual(len(self.reads()), before + 2)
        self.assertEqual(self.ready(), [])

    def test_grip_loss_during_deferred_success_still_recovers_immediately(self):
        self.make()
        self.begin_lift()
        self.assertTrue(self.observe(250, allow_success=False)['success_ready'])
        self.io.positions[5] = self.plan['close'][5] + 29
        state = self.step(50, allow_success=False)
        self.assertEqual(state['status'], 'recovering')
        self.assertFalse(state['success_ready'])
        self.assertIsNone(state['grasp_confirmed'])
        self.assertIs(state['grip_confirmation']['grasp_confirmed'], False)
        self.assertEqual(state['recovery_reason'], 'grip_lost')
        self.assertEqual(len(self.ready()), 1)

    def test_target_change_during_deferred_success_is_not_swallowed_by_terminal_state(self):
        self.make()
        self.begin_lift()
        self.assertTrue(self.observe(250, allow_success=False)['success_ready'])
        state = self.step(50, version=8, allow_success=True)
        self.assertEqual(state['status'], 'recovering')
        self.assertFalse(state['success_ready'])
        self.assertEqual(state['target_version'], 7)
        self.assertEqual(state['recovery_reason'], 'target_version_changed')
        self.assertEqual(len(self.ready()), 1)

    def test_default_success_gate_preserves_existing_callers(self):
        self.make()
        self.begin_lift()
        for _ in range(5):
            self.clock.advance(50)
            state = self.execution.step()
        self.assertEqual(state['status'], 'succeeded')
        self.assertIs(state['grasp_confirmed'], True)

    def test_deferred_success_readiness_clears_on_unknown_band_exit_or_arrival_loss(self):
        for invalidation in ('missing_feedback', 'gripper_band_exit', 'arrival_lost'):
            with self.subTest(invalidation=invalidation):
                self.make()
                self.begin_lift()
                self.assertTrue(self.observe(250, allow_success=False)['success_ready'])
                if invalidation == 'missing_feedback':
                    self.io.fail_read = True
                elif invalidation == 'gripper_band_exit':
                    self.io.positions[5] += 2
                else:
                    self.io.positions[1] += 11
                state = self.step(50, allow_success=False)
                self.assertEqual(state['status'], 'running')
                self.assertFalse(state['success_ready'])
                self.assertEqual(self.ready(), [])
                if invalidation == 'missing_feedback':
                    self.assertEqual(state['feedback_status'], 'unknown')
                    self.assertIsNone(state['grasp_confirmed'])

    def test_timers_work_across_board_tick_wrap(self):
        self.make(clock=Clock(start=980, period=1024))
        self.execution.invalidate_target()
        self.step()
        self.assertFalse(self.step(499)['can_reobserve'])
        self.assertTrue(self.step(1)['can_reobserve'])


if __name__ == '__main__':
    unittest.main()
