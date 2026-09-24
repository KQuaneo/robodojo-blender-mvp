"""Fast unit checks; no Blender, random draws or held-out data access."""
import unittest
from run_a import outcome

class OutcomeTests(unittest.TestCase):
    def result(self, reason, gates=None):
        return dict(termination_reason=reason, grasp_gates=gates or [])

    def test_legacy_task_failure(self):
        self.assertEqual(outcome(self.result('task_or_acceptance_failure'), []), 'task_failure')

    def test_success(self):
        self.assertEqual(outcome(self.result('success'), []), 'success')

    def test_invalid_initialization_precedence(self):
        self.assertEqual(outcome(self.result('invalid_initialization'), [{'frame':1}]), 'invalid_initialization')

    def test_earlier_collision_is_not_hidden_by_gate(self):
        result = self.result('missed_pickup', [dict(frame=65, passed=False)])
        self.assertEqual(outcome(result, [{'frame':64}]), 'collision')
        self.assertEqual(outcome(result, []), 'missed_pickup')

if __name__ == '__main__':
    unittest.main()
