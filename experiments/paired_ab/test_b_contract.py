"""Pure contract tests, with no Blender execution or layout generation."""
import ast
from pathlib import Path
import unittest

source = ast.parse((Path(__file__).parent / 'plan_b.py').read_text())
helpers = ast.Module(body=[n for n in source.body if isinstance(n,ast.FunctionDef)
                          and n.name in ('weight','stage')],type_ignores=[])
namespace = {}
exec(compile(helpers,'plan_b_helpers','exec'),namespace)
weight,stage = namespace['weight'],namespace['stage']

class ContractTests(unittest.TestCase):
    def test_broom_gate_and_handoff_boundaries(self):
        for frame,expected in ((1,0),(35,1),(55,1),(65,1),(115,1),(180,0),(190,0),(240,0)):
            self.assertEqual(weight(frame,1,35,115,180),expected)

    def test_pan_grasp_hold_and_home(self):
        for frame,expected in ((225,0),(250,1),(270,1),(880,1),(900,1),(1000,0)):
            self.assertEqual(weight(frame,225,250,900,1000),expected)

    def test_smooth_bounded_weights(self):
        values = [weight(f/8,1,35,115,180) for f in range(8001)]
        self.assertTrue(all(0<=x<=1 for x in values))
        self.assertLess(max(abs(a-b) for a,b in zip(values,values[1:])),.01)

    def test_stages(self):
        self.assertEqual(stage(65),'pickup_planning_failure')
        self.assertEqual(stage(190),'handoff_planning_failure')
        self.assertEqual(stage(270),'pan_planning_failure')
        self.assertEqual(stage(230,[['Right_link6','broom_shovel']]),'transition_to_sweep_failure')
        self.assertEqual(stage(900),'transition_to_sweep_failure')

if __name__ == '__main__':
    unittest.main()
