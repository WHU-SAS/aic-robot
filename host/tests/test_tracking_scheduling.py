"""Costed fake-clock regressions for the real TrackingTask scheduler.

Costs below are synthetic sensitivity inputs, not measured camera/CV CPU time
or physical robot latency. Only image, clock, motor and fine-work callbacks are
instrumented; scheduling and command coalescing use the production task.
"""
import unittest

from host.tests import test_robot_tracking as fixtures


class TrackingSchedulingTests(unittest.TestCase):
    def costed_fixture(self):
        # Keep the imported TestCase inside its module so unittest does not
        # discover and execute its entire suite again in this test module.
        fixture = fixtures.RobotTrackingTests()
        fixture.setUp()
        fixture.x = 650
        fixture.coarse_cost_ms = 110
        fixture.motor.request_cost_ms = 24
        fixture.fine_cost_ms = 20
        fixture.fine_slices = 16
        task = fixture.task
        self.addCleanup(task.close)
        trace = dict(fine=[], commands=[], frame_motor=[])
        capture = task.capture

        def costed_capture():
            image, frame = capture()
            fixture.clock.advance(20)
            return image, frame

        task.capture = costed_capture
        track = task.track

        def costed_track(image, frame):
            # The deliberately overdue load exercises driving/fine scheduling.
            # Restore the original fake60ms load after handoff so this test's
            # unrelated arm polling window can collect consecutive samples.
            fixture.coarse_cost_ms = 60 if task.arm is not None else 110
            return track(image, frame)

        task.track = costed_track
        advance = fixture.Fine.advance

        def measured_advance(session):
            started = fixture.clock.now()
            running = bool(fixture.motor.run)
            advance(session)
            trace['fine'].append(dict(started=started,
                capture=session.frame['capture_timestamp_ms'],
                observation_age=started-task._last_capture,
                running=running, completed=session.phase is None,
                finished=fixture.clock.now()))

        fixture.Fine.advance = measured_advance
        command = task._command

        def measured_command(value):
            sent = fixture.clock.now()
            before_state = task.state
            before_fields = dict(task.state)
            before_reply = task._last_motor_reply
            before_motion = task._motion_sent_at
            count = len(fixture.motor.commands)
            result = command(value)
            transmitted = len(fixture.motor.commands) != count
            trace['commands'].append(dict(command=value, intended_at=sent,
                transmitted=transmitted,
                sent_at=fixture.motor.commands[-1][0] if transmitted else None,
                motion_age=None if before_motion is None else sent-before_motion,
                same_state_object=result is before_state,
                unchanged_state=task.state == before_fields,
                unchanged_reply_stamp=task._last_motor_reply == before_reply))
            return result

        task._command = measured_command
        return fixture, trace

    def step(self, fixture, trace):
        before = len(fixture.observation_rows)
        fixture.task.step()
        self.assertIsNone(fixture.task.fault, fixture.task.snapshot())
        if len(fixture.observation_rows) != before:
            trace['frame_motor'].append(dict(fixture.task._frame_motor))
        fixture.clock.advance(fixture.task.idle_delay_ms())

    def until(self, fixture, trace, predicate, limit=1800):
        for unused in range(limit):
            self.step(fixture, trace)
            if predicate():
                return
        self.fail(fixture.task.snapshot())

    def test_overdue_moving_cycle_advances_fine_and_finishes_it_before_STOP(self):
        fixture, trace = self.costed_fixture()
        # Capture20 + STATUS24 + coarse110 already exceeds the normal100ms
        # observation target. Once fine starts, coarse capture is paused so
        # this overdue deadline cannot starve fine or consume its budget.
        self.until(fixture, trace, lambda: any(row['completed'] for row in trace['fine']))
        first = trace['fine'][0]
        complete = next(row for row in trace['fine'] if row['completed'])
        self.assertGreaterEqual(first['observation_age'], fixture.task.period)
        self.assertTrue(first['running'])
        self.assertTrue(complete['running'])
        self.assertFalse(any(command == 'STOP' for at, command in fixture.motor.commands))
        self.assertEqual(fixture.created_fines[0].latest['capture_timestamp_ms'], first['capture'])
        self.assertEqual(fixture.task.metrics['fine_started'], 1)
        self.assertFalse(any(first['capture'] < row['at'] < complete['finished']
                             for row in fixture.observation_rows))
        self.assertEqual(len(trace['fine']), fixture.fine_slices)
        # Synthetic fine work plus at most two fake24ms UART renewals. This is
        # a scheduler regression bound, not a measured K230 latency claim.
        self.assertLessEqual(complete['finished']-first['started'],
                             fixture.fine_slices*fixture.fine_cost_ms+2*24)
        self.until(fixture, trace, lambda: fixture.task.status == 'succeeded')
        first_stop = next(at for at, command in fixture.motor.commands if command == 'STOP')
        self.assertLess(complete['finished'], first_stop)
        self.assertEqual(fixture.task.metrics['arm_attempts'], 1)

    def test_coalescing_keeps_capture_STATUS_lease_and_real_reply_timestamps(self):
        fixture, trace = self.costed_fixture()
        self.until(fixture, trace, lambda: any(row['completed'] for row in trace['fine']))
        skipped = [row for row in trace['commands'] if not row['transmitted']]
        self.assertTrue(skipped)
        self.assertGreater(fixture.task.metrics['commands_coalesced'], 0)
        self.assertTrue(all(row['same_state_object'] and row['unchanged_state'] and
                            row['unchanged_reply_stamp'] for row in skipped))
        self.assertTrue(all(row['command'].startswith('SPEED,') or row['command'] == 'HB'
                            for row in skipped))
        rows = fixture.observation_rows
        self.assertEqual(len(rows), len(trace['frame_motor']))
        moving = [(row, sample) for row, sample in zip(rows, trace['frame_motor'])
                  if row['run'] or row['phase'] == 'moving']
        self.assertGreater(len(moving), 10)
        for row, sample in moving:
            self.assertGreaterEqual(row['motor_received'], row['at'])
            self.assertEqual(sample['_k230_received_ms'], row['motor_received'])
            self.assertEqual(sample['ms'], row['motor_ms'])
            next_capture = next((other['at'] for other in rows if other['at'] > row['at']),
                                fixture.clock.now()+1)
            statuses = [at for at, command in fixture.motor.commands
                        if command == 'STATUS' and row['at'] <= at < next_capture]
            handoff = fixture.task.metrics.get('last_moving_handoff') or {}
            immediate = handoff.get('status_sent_ms')
            # The accepted moving result now obtains its own t2 feedback and
            # sends the distance command BEFORE the next coarse capture.
            self.assertEqual(len(statuses), 1 + int(immediate in statuses))
        renewals = [at for at, command in fixture.motor.commands
                    if command.split(',')[0] in ('FOLLOW', 'SPEED', 'HB', 'ARC')]
        self.assertGreater(len(renewals), 2)
        gaps = [b-a for a, b in zip(renewals, renewals[1:])]
        gaps.append(fixture.clock.now()-renewals[-1])
        self.assertLess(max(gaps), 1000)  # Fake-clock lease coverage, not a hardware bound.

    def test_changed_direction_and_STOP_bypass_coalescing_in_the_same_cycle(self):
        fixture, trace = self.costed_fixture()
        self.until(fixture, trace, lambda: any(command.startswith('FOLLOW,')
            for at, command in fixture.motor.commands))
        initial = next(command.split(',')[1:] for at, command in fixture.motor.commands
                       if command.startswith('FOLLOW,'))
        # A small current-image lateral change stays within target association
        # and produces a new steering pair while the old lease is still young.
        fixture.y += 15
        self.until(fixture, trace, lambda: any(
            row['command'].startswith('SPEED,') and row['command'].split(',')[1:] != initial
            for row in trace['commands']))
        changed = next(row for row in trace['commands'] if
            row['command'].startswith('SPEED,') and row['command'].split(',')[1:] != initial)
        self.assertLess(changed['motion_age'], fixture.task.motion_refresh)
        self.assertTrue(changed['transmitted'])
        self.assertEqual(changed['sent_at'], changed['intended_at'])
        # Search entry must stop an owned old forward route, even immediately
        # after an acknowledged steering update; it must not throttle STOP.
        fixture.task.drive.request_search('left')
        self.until(fixture, trace, lambda: any(row['command'] == 'STOP'
                                             for row in trace['commands']))
        stopped = next(row for row in trace['commands'] if row['command'] == 'STOP')
        self.assertLess(stopped['motion_age'], fixture.task.motion_refresh)
        self.assertTrue(stopped['transmitted'])
        self.assertEqual(stopped['sent_at'], stopped['intended_at'])
        self.assertFalse(fixture.motor.run)


if __name__ == '__main__':
    unittest.main()
