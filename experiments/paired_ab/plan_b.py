"""Bounded scene-conditioned full-path retargeting; reject, never fallback."""
import argparse
import hashlib
import itertools
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import bpy
import numpy as np
from mathutils import Euler, Quaternion, Vector
import build_official_scene as b
import plan_collision_scene as p
from collision_geometry import CollisionWorld, intersects

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def curves(obj):
    result = {}
    action = obj.animation_data.action if obj.animation_data else None
    if action:
        for layer in action.layers:
            for strip in layer.strips:
                for bag in strip.channelbags:
                    for fc in bag.fcurves:
                        result[fc.data_path, fc.array_index] = fc
    return result

def action_signature(obj):
    return {str(k):[(list(p.co),p.interpolation) for p in fc.keyframe_points]
            for k,fc in curves(obj).items() if k[0]!='delta_location'}

def weight(t, start, full, end_full, end):
    def smooth(x):
        x = min(1., max(0., x))
        return x*x*(3-2*x)
    if t <= start or t >= end:
        return 0.
    if t < full:
        return smooth((t-start)/(full-start))
    if t <= end_full:
        return 1.
    return 1-smooth((t-end_full)/(end-end_full))

def stage(t, pairs=None):
    if 215 <= t <= 240 and pairs and any('Right_' in str(x) for x in pairs):
        return 'transition_to_sweep_failure'
    if t <= 115:
        return 'pickup_planning_failure'
    if t <= 225:
        return 'handoff_planning_failure'
    if t <= 270:
        return 'pan_planning_failure'
    return 'transition_to_sweep_failure'

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--trial', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args(sys.argv[sys.argv.index('--')+1:])
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    started = time.time()
    manifest = json.loads(Path(args.manifest).read_text())
    trial = next(t for t in manifest['trials'] if t['id'] == args.trial)
    assert trial['seed'] is None or trial['seed'] in range(20)
    for name, expected in manifest['artifacts'].items():
        assert sha(ROOT / name) == expected
    report = dict(id=args.trial, seed=trial['seed'], status='planning',
                  manifest_sha256=sha(args.manifest), offsets_m=trial['offsets_m'],
                  method='B_local_retarget_v1', checks=0, no_fallback=True)
    def finish(outcome, **extra):
        report.update(status='completed', outcome=outcome, duration_s=time.time()-started, **extra)
        (out / 'plan.json').write_text(json.dumps(report, indent=2))
        print('PLAN_RESULT', json.dumps(report), flush=True)

    scene = bpy.context.scene
    scene.frame_set(1)
    b.linear_keyframes()
    def mat(name):
        return np.array(bpy.data.objects[name].matrix_world)
    reference = json.loads((ROOT / 'output_smooth/joint_trace.json').read_text())
    cal = json.loads((ROOT / 'output_smooth/grasp_calibration.json').read_text())
    ee = [np.array(x) for x in cal['tool_to_flange']]
    robots = []
    for side in ('Left', 'Right'):
        joints = {j.name:bpy.data.objects[side+'_'+j.name] for j in b.ARM_JOINTS+b.GRIPPER_JOINTS}
        robots.append(dict(prefix=side, joints=joints, base_np=mat(side+'_root')))
    nominal_pan = mat('broom_shovel')
    pan_to_flange = np.linalg.inv(nominal_pan) @ b.fk(np.array(reference[269]['q'][0]), robots[0]['base_np'])
    report['pan_to_flange'] = pan_to_flange.tolist()
    joint_curves = [[curves(r['joints'][j.name]) for j in b.ARM_JOINTS] for r in robots]
    grip_curves = [curves(r['joints']['joint7']) for r in robots]
    def read_q(t, arm, source_curves):
        anchors = reference[min(999, max(0, int(t)-1))]['q'][arm]
        angles = []
        for i, spec in enumerate(b.ARM_JOINTS):
            fc = source_curves[arm][i]
            rot = Quaternion([fc['rotation_quaternion', k].evaluate(t) for k in range(4)]).normalized()
            variable = Euler(spec.rpy, 'XYZ').to_quaternion().conjugated() @ rot
            a = 2*math.atan2(float(np.dot(np.array([variable.x, variable.y, variable.z]), spec.axis)), variable.w)
            a += 2*math.pi*round((anchors[i]-a)/(2*math.pi))
            angles.append(a)
        return np.array(angles)
    def read_grip(t, arm):
        return float(grip_curves[arm]['location', 1].evaluate(t)-b.GRIPPER_JOINTS[0].xyz[1])
    times = {1+k/4 for k in range(3997)}
    for arm in joint_curves:
        for joint in arm:
            for fc in joint.values():
                times.update(float(k.co.x) for k in fc.keyframe_points if 1 <= k.co.x <= 1000)
    times = sorted(times)
    ref_q = {t:[read_q(t,i,joint_curves) for i in (0,1)] for t in times}
    ref_grips = {t:[read_grip(t,i) for i in (0,1)] for t in times}
    # Initialize once for planning, then save this initialization in accepted scenes.
    driver = bpy.data.objects['broom_driver']
    frozen_actions = {obj.name:action_signature(obj) for obj in
                      [driver]+list(robots[1]['joints'].values())}
    for name, delta in trial['offsets_m'].items():
        if name == 'broom_driver':
            driver.delta_location = Vector(delta)
        else:
            bpy.data.objects[name].location += Vector(delta)
    bpy.context.view_layer.update()
    names = [s+'_'+n for s in ('Left','Right') for n in ['base_link']+[f'link{i}' for i in range(1,9)]]
    movable = ['broom','broom_shovel','cube_0','cube_1','cube_2']
    names += movable+['Table']
    world = CollisionWorld([bpy.data.objects[n] for n in names])
    initial = {n:mat(n) for n in names}
    initial_driver = mat('broom_driver')
    errors = []
    hits = world.check(initial)
    if hits:
        errors.append(dict(robot_intersections=hits))
    for a,c in itertools.combinations(movable,2):
        if intersects(world.geometry[a],initial[a],world.geometry[c],initial[c]):
            errors.append(dict(object_intersection=[a,c]))
    table_lo, table_hi = world.geometry['Table'].bounds(initial['Table'])
    for n in movable:
        pts = world.geometry[n].vertices @ initial[n][:3,:3].T + initial[n][:3,3]
        lo,hi = pts.min(0),pts.max(0)
        if np.any(lo[:2]<table_lo[:2]) or np.any(hi[:2]>table_hi[:2]) or lo[2]<table_hi[2]-.002:
            errors.append(dict(invalid_bounds=n,minimum=lo.tolist(),maximum=hi.tolist()))
    if errors:
        finish('invalid_initialization', initialization_errors=errors)
        return
    offset_b = np.array(trial['offsets_m']['broom_driver'])
    offset_p = np.array(trial['offsets_m']['broom_shovel'])
    new_q = {}
    previous = None
    previous_t = None
    max_residual = 0.
    for t in times:
        q = [x.copy() for x in ref_q[t]]
        delta = weight(t,1,35,115,180)*offset_b + weight(t,225,250,900,1000)*offset_p
        if np.linalg.norm(delta) > 0:
            target = b.fk(q[0],robots[0]['base_np'])
            target[:3,3] += delta
            accepted = None
            errors = []
            for seed in ([q[0],previous[0]] if previous is not None else [q[0]]):
                candidate, err = p.single_ik(target, robots[0], seed)
                candidate = q[0] + (candidate-q[0]+math.pi)%(2*math.pi)-math.pi
                limited = bool(np.all(np.abs(candidate[:5])<=10) and abs(candidate[5])<=3.14)
                continuous = previous is None or np.max(np.abs(candidate-previous[0])) <= .48*(t-previous_t)+1e-6
                errors.append(dict(position_m=err[0],rotation_rad=err[1],limits=limited,continuity=bool(continuous)))
                if err[0]<=.0003 and err[1]<=.003 and limited and continuous:
                    accepted = candidate
                    max_residual = max(max_residual,err[0])
                    break
            if accepted is None:
                finish(stage(t), failure=dict(frame=t,type='IK_or_continuity',attempts=errors))
                return
            q[0] = accepted
        new_q[t] = q
        previous,previous_t = q,t
    # Keep right action untouched. Zero-offset control also keeps left untouched.
    if np.linalg.norm(offset_b)+np.linalg.norm(offset_p)>0:
        for spec in b.ARM_JOINTS:
            robots[0]['joints'][spec.name].animation_data_clear()
        for t in times:
            b.set_joint_pose(robots[0],new_q[t][0],ref_grips[t][0],t)
        b.linear_keyframes()
    # Delta affects only unheld initialization, never world-fixed park/sweep.
    for t, delta in [(1,offset_b),(64,offset_b),(65,np.zeros(3)),(1000,np.zeros(3))]:
        driver.delta_location = Vector(delta)
        driver.keyframe_insert('delta_location',frame=t)
    for (path,index),fc in curves(driver).items():
        if path == 'delta_location':
            for key in fc.keyframe_points:
                key.interpolation = 'CONSTANT'
    actual_curves = [[curves(r['joints'][j.name]) for j in b.ARM_JOINTS] for r in robots]
    samples = sorted(set(times) | {(a+c)/2 for a,c in zip(times,times[1:])})
    previous = None
    previous_t = None
    max_step_rate = 0.
    min_tool_table_gap = float('inf')
    max_frozen_sweep_error = 0.
    for t in samples:
        q = [read_q(t,i,actual_curves) for i in (0,1)]
        grips = [read_grip(t,i) for i in (0,1)]
        if any(np.any(np.abs(x[:5])>10+1e-6) or abs(x[5])>3.14+1e-6 for x in q) or any(g<0 or g>.044 for g in grips):
            finish(stage(t),failure=dict(frame=t,type='joint_limit'))
            return
        if previous is not None:
            rate = max(float(np.max(np.abs(q[i]-previous[i])))/(t-previous_t) for i in (0,1))
            max_step_rate = max(max_step_rate,rate)
            if max(float(np.max(np.abs(q[i]-previous[i]))) for i in (0,1)) > .48*(t-previous_t)+1e-5:
                finish(stage(t),failure=dict(frame=t,type='continuity',rad_per_frame=rate))
                return
        previous,previous_t = q,t
        mats = dict(initial)
        for i in (0,1):
            mats.update(p.link_matrices(robots[i],q[i],grips[i]))
        owner = 0 if 65<=t<190 else (1 if 190<=t<926 else None)
        if owner is not None:
            mats['broom'] = b.fk(q[owner],robots[owner]['base_np']) @ np.linalg.inv(ee[owner])
        elif t<65:
            mats['broom'] = initial_driver
        else:
            # Static parked broom after release; reference right flange at 925.
            mats['broom'] = b.fk(np.array(reference[924]['q'][1]),robots[1]['base_np']) @ np.linalg.inv(ee[1])
        if 240<=t<926:
            expected = b.fk(read_q(t,1,joint_curves),robots[1]['base_np']) @ np.linalg.inv(ee[1])
            max_frozen_sweep_error = max(max_frozen_sweep_error,float(np.max(np.abs(mats['broom']-expected))))
        hits = world.check(mats)
        if hits:
            finish(stage(t,hits),failure=dict(frame=t,type='collision',pairs=hits))
            return
        if t<240:
            tool = mats['broom']
            bottom = float(np.min(world.geometry['broom'].vertices @ tool[2,:3] + tool[2,3]))
            gap = bottom-float(table_hi[2])
            min_tool_table_gap = min(min_tool_table_gap,gap)
            if gap < -.0001:
                finish(stage(t),failure=dict(frame=t,type='tool_table_penetration',gap_m=gap))
                return
            for other in ('broom_shovel','cube_0','cube_1','cube_2'):
                if intersects(world.geometry['broom'],tool,world.geometry[other],mats[other]):
                    finish(stage(t),failure=dict(frame=t,type='tool_environment_collision',pairs=[['broom',other]]))
                    return
        report['checks'] += 1
        if report['checks']%800 == 0:
            print('PLAN_AUDIT',t,report['checks'],flush=True)
    assert max_frozen_sweep_error < 1e-10
    assert all(action_signature(bpy.data.objects[n]) == signature for n,signature in frozen_actions.items())
    trace = [dict(frame=f,q=[read_q(f,i,actual_curves).tolist() for i in (0,1)],
                  grips=[read_grip(f,i) for i in (0,1)]) for f in range(1,1001)]
    (out / 'joint_trace.json').write_text(json.dumps(trace))
    scene.frame_set(1)
    bpy.ops.wm.save_as_mainfile(filepath=str(out / 'planned.blend'),compress=True)
    finish('planned', max_ik_position_error_m=max_residual,
           max_joint_rate_rad_per_frame=max_step_rate,min_acquisition_tool_table_gap_m=min_tool_table_gap,
           frozen_sweep_pose_error=max_frozen_sweep_error,
           right_and_driver_reference_actions_unchanged=True,
           files={n:sha(out/n) for n in ('joint_trace.json','planned.blend')})

if __name__ == '__main__':
    main()
