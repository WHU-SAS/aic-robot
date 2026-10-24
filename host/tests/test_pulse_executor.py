import unittest
from device.k230.car.pulse_executor import PulseExecutor


class Link:
    def __init__(self):
        self.running = False; self.reason = 0; self.commands = []; self.failure = None
        self.ms = 0; self.encoder = 0
    def state(self):
        return {'protocol_version': 2, 'run': int(self.running),
                'pwm': 400 if self.running else 0, 'reason': self.reason,
                'ms': self.ms, 'encoder_a': self.encoder, 'encoder_b': -self.encoder}
    def expect(self, command):
        self.commands.append(command)
        self.ms += 100
        if self.failure == command: raise TimeoutError('simulated lost ACK')
        if command == 'STOP': self.running = False; self.reason = 1
        return self.state()
    def drive(self, *args):
        self.commands.append(('DRIVE', args)); self.running = True
        if self.failure == 'DRIVE': raise TimeoutError('simulated lost DRIVE ACK')
        return self.state()


class PulseExecutorTests(unittest.TestCase):
    def setUp(self):
        self.link = Link()
        self.profiles = {'forward_coarse': {'left_pwm': 800, 'right_pwm': 800, 'duration_ms': 300}}
        self.executor = PulseExecutor(self.link, self.profiles)
        self.plan = {'intent': 'forward_coarse', 'observation_timestamp_ms': 100}

    def test_motion_requires_stationary_sync_and_explicit_parameters(self):
        with self.assertRaises(RuntimeError): self.executor.submit(self.plan, 100)
        self.assertEqual(self.link.commands, [])
        self.executor.synchronize(); self.executor.profiles = {}
        with self.assertRaises(ValueError): self.executor.submit(self.plan, 100)
        self.assertNotIn('DRIVE', self.link.commands)

    def test_one_observation_can_only_start_one_pulse(self):
        self.executor.synchronize(); self.executor.submit(self.plan, 100)
        self.assertFalse(self.executor.can_localize)
        with self.assertRaises(RuntimeError): self.executor.submit(self.plan, 100)
        self.link.running = False; self.link.reason = 5
        self.assertEqual(self.executor.poll()['status'], 'settling')
        self.assertFalse(self.executor.can_localize)
        self.link.encoder += 10
        self.assertEqual(self.executor.poll()['status'], 'settling')
        self.assertEqual(self.executor.poll()['status'], 'settling')
        self.assertEqual(self.executor.poll()['status'], 'pulse_complete')
        self.assertTrue(self.executor.can_localize)
        with self.assertRaises(ValueError): self.executor.submit(self.plan, 400)
        self.assertEqual(sum(isinstance(c, tuple) for c in self.link.commands), 1)

    def test_button_or_watchdog_stop_latches_and_never_restarts(self):
        for reason in (1, 2, 4):
            self.setUp(); self.executor.synchronize(); self.executor.submit(self.plan, 100)
            self.link.running = False; self.link.reason = reason
            self.assertEqual(self.executor.poll()['status'], 'halted')
            with self.assertRaises(RuntimeError):
                self.executor.submit(dict(self.plan, observation_timestamp_ms=450), 450)
            self.executor.release_halt()
            self.executor.submit(dict(self.plan, observation_timestamp_ms=500), 500)
            self.assertTrue(self.link.running)

    def test_lost_drive_ack_attempts_stop_and_consumes_the_observation(self):
        self.executor.synchronize(); self.link.failure = 'DRIVE'
        with self.assertRaises(TimeoutError): self.executor.submit(self.plan, 100)
        self.assertFalse(self.link.running); self.assertTrue(self.executor.halted)
        self.assertEqual(self.link.commands[-1], 'STOP')
        self.link.failure = None; self.executor.release_halt()
        with self.assertRaises(ValueError): self.executor.submit(self.plan, 200)

    def test_unacknowledged_stop_does_not_claim_stationary_or_allow_localization(self):
        self.executor.synchronize(); self.executor.submit(self.plan, 100)
        self.link.failure = 'STOP'
        result = self.executor.cancel()
        self.assertFalse(result['stop_acknowledged'])
        self.assertFalse(self.executor.can_localize)
        self.assertTrue(self.executor.halted)

    def test_stale_observation_never_writes_drive(self):
        self.executor.synchronize()
        with self.assertRaises(ValueError): self.executor.submit(self.plan, 1000)
        self.assertEqual(self.link.commands, ['STATUS'])

    def test_observation_expiring_during_status_never_starts_motion(self):
        self.executor.clock_ms = lambda: 700
        self.executor.synchronize()
        with self.assertRaises(ValueError): self.executor.submit(self.plan, 100)
        self.assertEqual(self.link.commands, ['STATUS', 'STATUS'])

    def test_acknowledged_stop_still_waits_for_encoder_stability(self):
        self.executor.synchronize(); self.executor.submit(self.plan, 100)
        self.assertTrue(self.executor.cancel()['stop_acknowledged'])
        self.assertFalse(self.executor.can_localize)
        self.assertEqual(self.executor.poll()['status'], 'settling')
        self.assertEqual(self.executor.poll()['status'], 'pulse_complete')
        self.assertTrue(self.executor.can_localize)

    def test_stationary_wait_handles_mcu_millisecond_wrap(self):
        self.executor.synchronize(); self.executor.submit(self.plan, 100)
        self.link.running = False; self.link.reason = 5
        self.link.ms = 0xFFFFFFFF - 150
        self.assertEqual(self.executor.poll()['status'], 'settling')
        self.link.ms = 100
        self.assertEqual(self.executor.poll()['status'], 'pulse_complete')

    def test_unexpected_restart_during_settling_attempts_stop(self):
        self.executor.synchronize(); self.executor.submit(self.plan, 100)
        self.link.running = False; self.link.reason = 5
        self.executor.poll()
        self.link.running = True
        with self.assertRaises(RuntimeError): self.executor.poll()
        self.assertTrue(self.executor.halted)
        self.assertFalse(self.link.running)


if __name__ == '__main__': unittest.main()
