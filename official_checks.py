"""Execute selected unmodified RoboDojo predicate bodies against Blender state.

This is a state adapter, not the Isaac Sim environment. The support-circle
branch is sufficient for the original OR group; failures on that branch alone
are conservatively reported as failures (the bbox alternative is not used).
"""
import ast
import hashlib
import os
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from mathutils import Matrix, Quaternion

UPSTREAM=Path(os.environ.get('ROBODOJO_ROOT',str(Path(__file__).resolve().parent.parent/'RoboDojo')))
SOURCE=UPSTREAM/'env/reward_manager/func_parser.py'
HELPERS=SOURCE.parents[2]/'utils'/'transformer.py'
METHODS=['_select_label','is_not_lift','is_A_on_B_left','is_axis_up',
         'is_A_in_B_support_circle','is_all_A_in_B_support_circle','all_robot_back_to_origin']

def quaternion_distance(a,b):
    a=np.asarray(a);b=np.asarray(b)
    return 2*np.arccos(np.clip(abs(np.dot(a,b))/(np.linalg.norm(a)*np.linalg.norm(b)),0,1))

def pose7(m):
    q=Matrix(np.asarray(m)[:3,:3].tolist()).to_quaternion()
    return np.r_[np.asarray(m)[:3,3],np.array(q)]

def load_predicates():
    namespace={'np':np,'safe_deepcopy_keep_callable':deepcopy,'cal_quat_dis':quaternion_distance}
    helper_tree=ast.parse(HELPERS.read_text())
    for node in helper_tree.body:
        if isinstance(node,ast.FunctionDef) and node.name in ['quat_to_mat','cal_two_axis_angle','check_1d','check_2d','_check_ragged_2d']:
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(HELPERS),'exec'),namespace)
    tree=ast.parse(SOURCE.read_text());methods={}
    for cls in tree.body:
        if not isinstance(cls,ast.ClassDef):continue
        for node in cls.body:
            if isinstance(node,ast.FunctionDef) and node.name in METHODS:
                exec(compile(ast.Module(body=[node],type_ignores=[]),str(SOURCE),'exec'),namespace)
                methods[node.name]=namespace[node.name]
    assert set(methods)==set(METHODS)
    return type('OfficialPredicates',(),methods)

class StateAdapter:
    def __init__(self,initial_pan,home_poses):
        self.pan_start=initial_pan
        self.parser=load_predicates()()
        p=self.parser;p.num_envs=1;p.layout_manager=self;p.robot_manager=self
        p.pre_state=[{'pan':{'pose':pose7(initial_pan)}}]
        self.robot_list=[SimpleNamespace(type='target',arm_name=str(i)) for i in range(2)]
        p.robot_origin_endpose=[{str(i):pose7(m) for i,m in enumerate(home_poses)}]
    def get_instance_name(self,label,**kw):return label
    def get_scene_object(self,inst_name,**kw):return self.poses.get(inst_name)
    def get_instance_pose(self,inst_name,**kw):
        p=pose7(self.poses[inst_name]);return p[:3],p[3:]
    def get_instance_metadata(self,**kw):return {}
    def get_support_points(self,**kw):
        center=self.poses['pan']@np.array([0,-.053,-.015,1])
        return [center[:3]],[.081]
    def get_real_endpose(self,robot):return [pose7(self.robot_poses[int(robot.arm_name)])]
    def check(self,poses,robots):
        self.poses=poses;self.robot_poses=robots;p=self.parser
        def call(name,**kw):return bool(getattr(p,name)({'env_idx':0,**kw})>=1)
        return {
            'pan_left_of_broom':call('is_A_on_B_left',label_A='pan',label_B='broom',x_threshold=.1),
            'pan_not_lifted':call('is_not_lift',label='pan',z_threshold=.01),
            'pan_upright':call('is_axis_up',label='pan',axis=[0,0,1],threshold=5),
            'all_in_support_circle':call('is_all_A_in_B_support_circle',label_A=['cube0','cube1','cube2'],label_B='pan',B_support_tag='broom_shovel/0'),
            'robots_home':call('all_robot_back_to_origin',pos_threshold=.15,rot_threshold=20),
        }

def source_fingerprint():return hashlib.sha256(SOURCE.read_bytes()).hexdigest()
