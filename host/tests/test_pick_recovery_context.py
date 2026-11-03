"""Recovery diagnostics are detached metadata; fake IO only, no hardware."""
import copy
import json
import unittest

from host.tests import test_arm_pick_execution as fixtures


class PickRecoveryContextTests(unittest.TestCase):
    def fixture(self):
        case = fixtures.PickExecutionTests()
        case.make()
        case.begin_lift(delta=61)
        return case

    def recovery_events(self, case):
        return [details for name, details in case.emitted if name == 'recovery_goal_sent']

    def test_context_changes_only_existing_event_metadata_not_io_or_recovery(self):
        baseline, diagnostic = self.fixture(), self.fixture()
        before_reads = len(diagnostic.reads())
        before_events = len(diagnostic.emitted)
        expected = baseline.execution.step(target_version=8)
        actual = diagnostic.execution.step(target_version=8, target_context={
            'old_version': 7, 'new_version': 8, 'reason': 'held_target_moved',
            'coarse': {'associated': False, 'center': [310., 240.]},
            'fk': {'frame_id': 42, 'age_ms': 10}, 'motor': {'moving': False}})
        self.assertEqual(actual, expected)
        self.assertEqual(diagnostic.io.events, baseline.io.events)
        self.assertEqual(len(diagnostic.reads()), before_reads)
        self.assertEqual(len(diagnostic.ready()), 1)
        self.assertEqual(len(diagnostic.emitted), before_events + 1)
        event = self.recovery_events(diagnostic)[0]
        self.assertEqual(event['reason'], 'target_version_changed')
        self.assertEqual(event['target_context']['reason'], 'held_target_moved')
        without_context = [(name, {k: v for k, v in payload.items() if k != 'target_context'})
                           for name, payload in diagnostic.emitted]
        self.assertEqual(without_context, baseline.emitted)
        self.assertNotIn('target_context', self.recovery_events(baseline)[0])
        json.dumps(diagnostic.emitted)

    def test_context_is_detached_before_ready_dispatch(self):
        case = self.fixture()
        context = {'reason': 'held_target_moved', 'frame': {'centers': [[1., 2.]]},
                   'flags': [True, None, 'present'], 'versions': (7, 8)}
        expected = copy.deepcopy(context)
        expected['versions'] = [7, 8]
        original_ready = case.io.send_ready2

        def ready_with_caller_mutation():
            context['reason'] = 'mutated_during_dispatch'
            context['frame']['centers'][0][0] = 999.
            return original_ready()

        case.io.send_ready2 = ready_with_caller_mutation
        case.execution.step(target_version=8, target_context=context)
        context['flags'].append(False)
        self.assertEqual(self.recovery_events(case)[0]['target_context'], expected)

    def test_same_valid_version_ignores_context_and_has_no_stale_metadata(self):
        case = self.fixture()
        case.execution.step(target_version=7, target_context={'reason': 'irrelevant'})
        self.assertEqual(self.recovery_events(case), [])
        self.assertIsNone(case.execution._recovery_target_context)
        case.execution.step(target_version=8)
        self.assertNotIn('target_context', self.recovery_events(case)[0])

    def test_same_version_already_invalidated_records_context_and_keeps_reason(self):
        baseline, diagnostic = self.fixture(), self.fixture()
        for case in (baseline, diagnostic):
            case.execution.invalidate_target('held_target_moved')
        expected = baseline.execution.step(target_version=7)
        actual = diagnostic.execution.step(target_version=7, target_context={'reason': 'same_version_revoke'})
        self.assertEqual(actual, expected)
        self.assertEqual(diagnostic.io.events, baseline.io.events)
        event = self.recovery_events(diagnostic)[0]
        self.assertEqual(event['reason'], 'held_target_moved')
        self.assertEqual(event['target_context'], {'reason': 'same_version_revoke'})
        self.assertEqual(len(diagnostic.emitted), len(baseline.emitted))

    def test_context_is_not_replaced_or_logged_twice_during_recovery(self):
        case = self.fixture()
        case.execution.step(target_version=8, target_context={'reason': 'first'})
        emitted = copy.deepcopy(case.emitted)
        operations = list(case.io.events)
        for after in (0, 100, 400, 50):
            case.clock.advance(after)
            case.execution.step(target_version=9, target_context={'reason': 'later'})
        self.assertEqual(case.emitted, emitted)
        self.assertEqual(case.io.events, operations)
        self.assertEqual(self.recovery_events(case)[0]['target_context'], {'reason': 'first'})

    def test_empty_context_is_preserved_and_input_is_not_modified(self):
        case = self.fixture()
        context = {}
        case.execution.step(target_version=8, target_context=context)
        self.assertEqual(context, {})
        self.assertEqual(self.recovery_events(case)[0]['target_context'], {})

    def test_invalid_diagnostic_values_do_not_prevent_original_recovery(self):
        cycle = {}
        cycle['cycle'] = cycle
        for context in ({'bad': object()}, {1: 'bad_key'}, cycle):
            with self.subTest(kind=str(type(next(iter(context.values()))))):
                baseline, diagnostic = self.fixture(), self.fixture()
                expected = baseline.execution.step(target_version=8)
                actual = diagnostic.execution.step(target_version=8, target_context=context)
                self.assertEqual(actual, expected)
                self.assertEqual(diagnostic.io.events, baseline.io.events)
                self.assertEqual(self.recovery_events(diagnostic)[0]['target_context'],
                                 {'capture_error': 'context_not_json_compatible'})
                json.dumps(diagnostic.emitted)

    def test_same_version_context_does_not_relabel_actual_grip_loss(self):
        case = self.fixture()
        case.io.positions[5] = case.plan['close'][5] + 29
        case.execution.step(target_version=7, target_context={'reason': 'irrelevant'})
        event = self.recovery_events(case)[0]
        self.assertEqual(event['reason'], 'grip_lost')
        self.assertNotIn('target_context', event)
        self.assertEqual(len(case.ready()), 1)


if __name__ == '__main__':
    unittest.main()
