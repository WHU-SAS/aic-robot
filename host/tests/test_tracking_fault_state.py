"""Real task fault latch: retain first cause and actual motor reply."""
import unittest

from host.tests import test_robot_tracking as fixtures


class TrackingFaultStateTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.RobotTrackingTests()
        self.fixture.setUp()
        self.task = self.fixture.task
        self.addCleanup(self.task.close)

    def test_fault_stops_and_no_longer_looks_like_active_search(self):
        self.task._command('FOLLOW,-20,20')
        self.task.decision = {'phase':'searching'}
        self.fixture.clock.advance(30)
        self.task._fault('motor_command', 'first ACK failure')
        first = self.task.snapshot()
        self.assertEqual(first['phase'], 'fault_wait')
        self.assertEqual(first['fault_wait'], 'motor_command: first ACK failure')
        self.assertEqual(first['metrics']['fault_state_before_stop']['run'], 1)
        self.assertEqual(first['metrics']['fault_stop_reply']['run'], 0)
        self.assertEqual(first['metrics']['fault_stop_reply']['reason'], 1)
        original_reply = dict(first['metrics']['fault_stop_reply'])
        observations = first['metrics']['observations']
        commands = list(self.fixture.motor.commands)
        for unused in range(20):
            self.fixture.clock.advance(100)
            self.task.step()
        self.assertEqual(self.task.metrics['observations'], observations)
        self.assertEqual(self.fixture.motor.commands, commands)
        self.task._fault('tracking_io', 'IDE interrupt')
        second = self.task.snapshot()
        self.assertEqual(second['fault_wait'], first['fault_wait'])
        self.assertEqual(second['metrics']['fault_stop_reply'], original_reply)
        self.assertIn('IDE interrupt', second['metrics']['last_secondary_fault'])
        errors = [row for row in second['timeline'] if row['event']=='tracking_fault_wait']
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0]['details']['reason'], first['fault_wait'])

    def test_stop_failure_is_retained_without_claiming_a_motor_ACK(self):
        original = self.fixture.motor.expect
        def fail_stop(command):
            if command == 'STOP':
                raise OSError('STOP ACK missing')
            return original(command)
        self.fixture.motor.expect = fail_stop
        self.task._fault('fine_compute', 'original solve exception')
        self.assertEqual(self.task.snapshot()['phase'], 'fault_wait')
        self.assertNotIn('fault_stop_reply', self.task.metrics)
        self.assertEqual(self.task.metrics['fault_stop_error'], 'STOP ACK missing')
        self.fixture.motor.expect = original


if __name__ == '__main__':
    unittest.main()
