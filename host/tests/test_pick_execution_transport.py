"""Exercise the built K230 control module against fake UARTs, not just IO stubs."""
import contextlib
import copy
import io
import json
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tools.deploy.k230_pick import build_bundle
from host.tests.test_arm_pick_execution import PLAN, POLICY, Clock


class PickExecutionTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config, cls.files, _ = build_bundle()

    def harness(self):
        self.clock = Clock()
        self.events = []
        self.writes = []
        self.positions = list(PLAN['hover'])
        self.short_write = False
        self.write_error = None
        owner = self

        class Pins:
            UART1_TXD = 1
            UART1_RXD = 2
            def set_function(self, *args):
                owner.events.append(('pin', args))

        class Port:
            UART1 = 1
            EIGHTBITS = 8
            PARITY_NONE = None
            STOPBITS_ONE = 1
            def __init__(self, *args, **kwargs):
                owner.events.append(('open', args, kwargs))
            def write(self, packet):
                owner.writes.append((owner.clock.now(), packet))
                if owner.write_error is not None:
                    raise owner.write_error
                return len(packet) - 1 if owner.short_write else len(packet)
            def deinit(self):
                owner.events.append(('deinit',))

        ns = {}
        fake_time = SimpleNamespace(ticks_ms=self.clock.now, ticks_diff=self.clock.diff,
                                    sleep_ms=self.clock.advance)
        with patch.dict(sys.modules, {'machine': SimpleNamespace(UART=Port, FPIOA=Pins),
                                     'time': fake_time, 'ujson': json}):
            exec(self.files['control.py'].decode(), ns)

        def read(port, axis, address, size):
            owner.events.append(('read', axis, address, size))
            owner.clock.advance(1)
            if (address, size) == (40, 1):
                return b'\x01'  # Already powered: ready2 must not write torque.
            self.assertEqual((address, size), (56, 2))
            value = owner.positions[axis - 1]
            return bytes((value & 255, value >> 8))
        ns['read_checked'] = read
        ns['print'] = lambda *args, **kwargs: None
        self.ns = ns
        self.control = copy.deepcopy(self.config['control'])
        self.ready = copy.deepcopy(self.config['drive_ready_control'])
        return ns['open_pick_execution'](self.control, self.ready, copy.deepcopy(PLAN), POLICY, 7)

    def ids(self, packet):
        self.assertEqual(packet[:3], b'\xff\xff\xfe')
        self.assertEqual(packet[4:7], bytes((0x83, 41, 7)))
        self.assertEqual(len(packet), packet[3] + 4)
        self.assertEqual(sum(packet[2:]) & 255, 255)
        return list(packet[7:-1:8])

    def test_construct_is_inert_and_first_goal_matches_original_packet_profiles(self):
        engine = self.harness()
        self.assertEqual(self.events, [])
        self.assertEqual(self.writes, [])
        expected = self.ns['sync_frame'](PLAN['hover'], self.control['profiles'])
        engine.step(7)
        self.assertEqual(self.writes, [(0, expected)])
        self.assertEqual(self.ids(expected), [1, 2, 3, 4, 5, 6])
        self.assertFalse(any(event[0] == 'read' for event in self.events))
        engine.close()
        engine.close()
        self.assertEqual(self.events.count(('deinit',)), 1)

    def assert_dispatch(self, engine, name, axes, seq):
        dispatch = engine.snapshot()['last_goal_dispatch']
        self.assertIsNotNone(dispatch)
        self.assertEqual(dispatch['seq'], seq)
        self.assertEqual(dispatch['name'], name)
        self.assertEqual(dispatch['axes'], axes)
        # The physical subset packet still records the complete stage goal.
        self.assertEqual(dispatch['goals'], PLAN[name])
        written_at, packet = self.writes[-1]
        self.assertEqual(self.ids(packet), axes)
        self.assertEqual(packet, self.ns['sync_frame'](
            PLAN[name], self.control['profiles'], axes))
        # Adapter success includes its original2ms TX drain, not just write().
        self.assertEqual(dispatch['sent_at_ms'], written_at + 2)
        self.assertEqual(dispatch['sent_at_ms'], self.clock.now())
        return copy.deepcopy(dispatch)

    def test_successful_real_packets_record_all_full_and_split_stage_dispatches(self):
        engine = self.harness()
        self.assertIsNone(engine.snapshot()['last_goal_dispatch'])
        self.assertEqual(self.writes, [])
        engine.step(7)
        hover = self.assert_dispatch(engine, 'hover', [1, 2, 3, 4, 5, 6], 1)
        engine.step(7)  # Six real adapter position reads pass hover.
        self.assertEqual(engine.snapshot()['last_goal_dispatch'], hover)
        engine.step(7)
        lead = self.assert_dispatch(engine, 'descend', [4], 2)
        self.clock.advance(POLICY['descend_axis4_lead_ms'] - 1)
        engine.step(7)
        self.assertEqual(engine.snapshot()['last_goal_dispatch'], lead)
        self.assertEqual(len(self.writes), 2)
        self.clock.advance(1)
        engine.step(7)
        self.assert_dispatch(engine, 'descend', [1, 2, 3, 5, 6], 3)
        self.assertEqual(self.writes[2][0] - lead['sent_at_ms'], 200)
        self.positions = list(PLAN['descend'])
        engine.step(7)
        engine.step(7)
        self.assert_dispatch(engine, 'close', [1, 2, 3, 4, 5, 6], 4)
        self.positions[5] = PLAN['close'][5] + 45
        for index in range(POLICY['gripper_stable_samples']):
            if index:
                self.clock.advance(POLICY['poll_interval_ms'])
            engine.step(7)
        self.assertEqual(engine.snapshot()['pending_stage'], 'lift')
        engine.step(7)
        self.assert_dispatch(engine, 'lift', [1, 2, 3, 4, 5, 6], 5)
        self.assertEqual(len(self.writes), 5)
        engine.close()
        # Sequence numbers belong to one execution, not to the UART module.
        fresh = self.harness()
        self.assertIsNone(fresh.snapshot()['last_goal_dispatch'])
        fresh.step(7)
        self.assert_dispatch(fresh, 'hover', [1, 2, 3, 4, 5, 6], 1)
        fresh.close()

    def test_first_packet_short_write_or_write_exception_does_not_record_success(self):
        for fault in ('short', 'exception'):
            with self.subTest(fault=fault):
                engine = self.harness()
                if fault == 'short':
                    self.short_write = True
                else:
                    self.write_error = OSError('fake UART write failure')
                state = engine.step(7)
                self.assertEqual(state['status'], 'fault')
                self.assertIsNone(state['last_goal_dispatch'])
                self.assertEqual(len(self.writes), 1)
                self.assertEqual(self.ids(self.writes[0][1]), [1, 2, 3, 4, 5, 6])
                self.assertEqual(self.clock.now(), 0)  # No successful drain.
                engine.step(8)
                self.assertEqual(len(self.writes), 1)
                self.assertIsNone(engine.snapshot()['last_goal_dispatch'])
                engine.close()

    def test_failed_descend_half_keeps_last_successful_real_packet_metadata(self):
        for half in ('axis4', 'remaining'):
            for fault in ('short', 'exception'):
                with self.subTest(half=half, fault=fault):
                    engine = self.harness()
                    engine.step(7)
                    engine.step(7)
                    if half == 'remaining':
                        engine.step(7)
                        self.clock.advance(POLICY['descend_axis4_lead_ms'])
                    previous = copy.deepcopy(engine.snapshot()['last_goal_dispatch'])
                    if fault == 'short':
                        self.short_write = True
                    else:
                        self.write_error = OSError('fake UART write failure')
                    state = engine.step(7)
                    self.assertEqual(state['status'], 'fault')
                    self.assertEqual(state['last_goal_dispatch'], previous)
                    self.assertEqual(previous['seq'], 2 if half == 'remaining' else 1)
                    axes = [1, 2, 3, 5, 6] if half == 'remaining' else [4]
                    self.assertEqual(self.ids(self.writes[-1][1]), axes)
                    self.assertEqual(self.writes[-1][1], self.ns['sync_frame'](
                        PLAN['descend'], self.control['profiles'], axes))
                    count = len(self.writes)
                    engine.step(8)
                    self.assertEqual(len(self.writes), count)
                    self.assertEqual(engine.snapshot()['last_goal_dispatch'], previous)
                    engine.close()

    def test_ready2_recovery_is_not_a_cooperative_stage_dispatch(self):
        for sent_hover in (False, True):
            with self.subTest(sent_hover=sent_hover):
                engine = self.harness()
                if sent_hover:
                    engine.step(7)
                previous = copy.deepcopy(engine.snapshot()['last_goal_dispatch'])
                engine.invalidate_target()
                state = engine.step(8)
                self.assertEqual(state['status'], 'recovering')
                self.assertEqual(state['last_goal_dispatch'], previous)
                self.assertEqual(self.writes[-1][1], self.ns['sync_frame'](
                    self.ready['goals'], self.ready['profiles']))
                self.assertEqual(self.ids(self.writes[-1][1]), [1, 2, 3, 4, 5, 6])
                engine.close()

    def test_invalid_late_stage_or_ready_config_fails_before_uart_access(self):
        self.harness()
        for variant in ('plan', 'ready', 'policy'):
            with self.subTest(variant=variant):
                plan = copy.deepcopy(PLAN)
                ready = copy.deepcopy(self.ready)
                policy = dict(POLICY)
                if variant == 'plan':plan['lift'][5] = 5000
                elif variant == 'ready':ready['goals'][3] = -1
                else:policy['lift_gripper_stable_ms'] = -1
                with self.assertRaises(ValueError):
                    self.ns['open_pick_execution'](self.control, ready, plan, policy, 7)
                self.assertEqual(self.events, [])
                self.assertEqual(self.writes, [])

    def test_recovery_reuses_uart_overwrites_wrist_and_times_from_actual_send(self):
        engine = self.harness()
        engine.step(7)  # hover packet
        engine.step(7)  # hover positions pass
        engine.step(7)  # axis4 only
        self.assertEqual(self.ids(self.writes[-1][1]), [4])
        # Invalidate inside the wrist lead: old remaining five must never send.
        engine.invalidate_target()
        engine.step(8)
        state = engine.snapshot()
        goal_time, packet = self.writes[-1]
        self.assertEqual(packet, self.ns['sync_frame'](self.ready['goals'], self.ready['profiles']))
        self.assertEqual(self.ids(packet), [1, 2, 3, 4, 5, 6])
        self.assertEqual(state['recovery_goal_sent_ms'], goal_time)
        self.assertEqual(len([e for e in self.events if e[0] == 'open']), 1)
        self.assertEqual(len([e for e in self.events if e[0] == 'read' and e[2] == 40]), 6)
        # Already-on torque: all actual writes are sync goal/profile frames.
        self.assertTrue(all(p[4] == 0x83 for _, p in self.writes))
        self.clock.elapsed = goal_time + 499
        self.assertFalse(engine.step(99)['can_reobserve'])
        self.clock.advance(1)
        self.assertTrue(engine.step(99)['can_reobserve'])
        self.assertEqual(len(self.writes), 3)
        engine.close()

    def test_short_uart_write_latches_fault_and_does_not_retry_or_release_torque(self):
        engine = self.harness()
        self.short_write = True
        self.assertEqual(engine.step(7)['status'], 'fault')
        engine.step(8)
        self.assertEqual(len(self.writes), 1)
        self.assertFalse(any(event[0] == 'read' for event in self.events))
        self.assertEqual(self.ids(self.writes[0][1]), [1, 2, 3, 4, 5, 6])
        engine.close()

    def test_profiles_and_ready_targets_are_frozen_at_construction(self):
        engine = self.harness()
        expected = self.ns['sync_frame'](self.ready['goals'], self.ready['profiles'])
        self.ready['goals'][0] = 2500
        self.ready['profiles'][0]['speed_raw'] = 1
        self.control['profiles'][0]['speed_raw'] = 1
        engine.invalidate_target()
        engine.step(7)
        self.assertEqual(self.writes[-1][1], expected)
        engine.close()


if __name__ == '__main__':unittest.main()
