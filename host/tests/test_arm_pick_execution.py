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

    def send_goals(self, goals, servo_ids):
        self.events.append(('goals', self.clock.now(), list(goals), list(servo_ids)))
        if self.fail_send:
            raise RuntimeError('fake short goal write')

    def read_positions(self, servo_ids):
        self.events.append(('read', self.clock.now(), list(servo_ids)))
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
        self.execution = PickExecution(self.plan, self.policy, self.io, 7,
                                       self.clock.now, self.clock.diff)
        return self.execution

    def step(self, after=0, version=7):
        self.clock.advance(after)
        return self.execution.step(target_version=version)

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
        self.step(150)
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
        self.assertNotEqual(self.step(150)['status'], 'succeeded')
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
        self.assertNotEqual(self.step(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_out_of_band_keeps_grip_and_restarts_fixed_band_timer(self):
        self.make()
        self.begin_lift()
        self.step(50)
        self.step(150)
        self.io.positions[5] += 2
        self.assertEqual(self.step(50)['status'], 'running')
        self.assertEqual(self.ready(), [])
        self.assertNotEqual(self.step(150)['status'], 'succeeded')
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
        self.step(100)
        self.assertEqual(self.step(50)['status'], 'succeeded')

    def test_lift_losing_arrival_resets_success_interval(self):
        self.make()
        self.begin_lift()
        self.step(50)
        self.step(150)
        self.io.positions[3] += 11
        self.assertNotEqual(self.step(50)['status'], 'succeeded')
        self.io.positions[3] -= 11
        self.step(50)
        self.assertNotEqual(self.step(150)['status'], 'succeeded')
        self.assertEqual(self.step(50)['status'], 'succeeded')

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
        self.step(150)
        self.io.fail_read = True
        unknown = self.step(50)
        self.assertNotEqual(unknown['status'], 'succeeded')
        self.assertEqual(unknown['feedback_status'], 'unknown')
        self.assertIsNone(unknown['grasp_confirmed'])
        self.assertEqual(self.ready(), [])
        self.step(50)
        self.assertNotEqual(self.step(150)['status'], 'succeeded')
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
        self.step(200)
        self.assertEqual(self.execution.snapshot()['status'], 'succeeded')
        before = list(self.io.events)
        self.step(500)
        self.assertEqual(self.io.events, before)

    def test_timers_work_across_board_tick_wrap(self):
        self.make(clock=Clock(start=980, period=1024))
        self.execution.invalidate_target()
        self.step()
        self.assertFalse(self.step(499)['can_reobserve'])
        self.assertTrue(self.step(1)['can_reobserve'])


if __name__ == '__main__':
    unittest.main()
