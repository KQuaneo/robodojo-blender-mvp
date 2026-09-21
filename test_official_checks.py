"""Run with Blender Python; includes negative controls for every success gate."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from official_checks import StateAdapter

pan=np.eye(4);home=[np.eye(4),np.eye(4)]
checker=StateAdapter(pan,home)
poses={'pan':pan.copy(),'broom':np.eye(4)}
poses['broom'][0,3]=.3
for i in range(3):
    poses[f'cube{i}']=np.eye(4);poses[f'cube{i}'][1,3]=-.053
assert all(checker.check(poses,home).values())
for name,obj,index,value in [('all_in_support_circle','cube0',(0,3),.3),
                            ('pan_left_of_broom','broom',(0,3),0),
                            ('pan_not_lifted','pan',(2,3),.02)]:
    altered={k:v.copy() for k,v in poses.items()};altered[obj][index]=value
    assert not checker.check(altered,home)[name],name
altered={k:v.copy() for k,v in poses.items()};altered['pan'][:3,:3]=np.diag([1,-1,-1])
assert not checker.check(altered,home)['pan_upright']
away=[m.copy() for m in home];away[0][0,3]=.2
assert not checker.check(poses,away)['robots_home']
print('PASS: positive state plus five independent negative controls')
