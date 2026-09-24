"""Analytical-rate regression tests using synthetic curves, no task simulation."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE.parents[1]),str(HERE)]
import motion

class Curve:
    def __init__(self,a,c,duration,interpolation='LINEAR'):
        self.a,self.c,self.duration=a,c,duration
        self.keyframe_points=[SimpleNamespace(co=SimpleNamespace(x=x),interpolation=interpolation)
                              for x in (1.,1.+duration)]
    def evaluate(self,t):return self.a+(self.c-self.a)*(t-1)/self.duration

def synthetic(a,c,duration,interpolation='LINEAR'):
    return [[{('rotation_quaternion',k):Curve(a[k],c[k],duration,interpolation) for k in range(4)}]]

class Rates(unittest.TestCase):
    def test_static(self):
        result=motion.rate_audit(synthetic([1,0,0,0],[1,0,0,0],1))
        self.assertTrue(result['passed'])
        self.assertEqual(result['maximum']['rate_rad_per_frame'],0)
    def test_exact_nlerp_maximum(self):
        angle=math.pi/2
        result=motion.rate_audit(synthetic([1,0,0,0],[math.cos(angle/2),math.sin(angle/2),0,0],10))
        self.assertAlmostEqual(result['maximum']['rate_rad_per_frame'],4*math.tan(angle/4)/10,places=12)
        self.assertTrue(result['passed'])
    def test_too_fast(self):
        self.assertFalse(motion.rate_audit(synthetic([1,0,0,0],[0,1,0,0],1))['passed'])
    def test_antipodal_degenerate(self):
        self.assertFalse(motion.rate_audit(synthetic([1,0,0,0],[-1,0,0,0],1))['passed'])
    def test_bezier_not_falsely_certified(self):
        with self.assertRaises(AssertionError):
            motion.rate_audit(synthetic([1,0,0,0],[1,0,0,0],1,'BEZIER'))

if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(Rates)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():raise RuntimeError('Analytical continuity tests failed')
