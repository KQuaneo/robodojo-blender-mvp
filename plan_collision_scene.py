"""Collision-filtered IK and calibrated, non-penetrating scripted grasp."""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
from mathutils import Matrix,Vector
import build_verified_scene as v
import build_official_scene as b
from collision_geometry import CollisionWorld,intersects

OUT=b.ROOT/'output_collision'
OUT.mkdir(exist_ok=True)

def link_matrices(robot,q,grip):
    out={robot['prefix']+'_base_link':robot['base_np']}
    t=robot['base_np'].copy()
    for angle,j in zip(q,b.ARM_JOINTS):
        t=t@b.np_transform(j.xyz,b.np_rpy(j.rpy))@b.np_transform(rotation=b.np_axis_angle(j.axis,angle))
        out[robot['prefix']+'_'+j.child]=t.copy()
    for j in b.GRIPPER_JOINTS:
        out[robot['prefix']+'_'+j.child]=t@b.np_transform(np.array(j.xyz)+np.array(j.axis)*grip)
    return out

class Planner:
    def __init__(self,robots,objects):
        self.robots=robots;self.objects=objects
        meshes=[o for r in robots for o in r['meshes'].values()]
        self.world=CollisionWorld(meshes+[objects['broom_visual'],objects['pan'],bpy.data.objects['Table']])
        self.fixed={n:np.array(o.matrix_world) for n,o in self.world.objects.items()}
        self.q=[np.zeros(6),np.zeros(6)];self.grips=[.03,.03]
        self.tool=None;self.frame=0;self.restarts=0
        self.warm=b.ROOT/'output_collision'/'ik_seeds.json'
        self.seeds=json.loads(self.warm.read_text()) if self.warm.exists() else {}
        self.saved={}
        self.carry=None;self.ee=None
    def matrices(self,arm=None,q=None,grip=None):
        out=dict(self.fixed)
        for i,r in enumerate(self.robots):
            out.update(link_matrices(r,q if arm==i else self.q[i],grip if arm==i and grip is not None else self.grips[i]))
        out['broom']=self.tool
        if self.carry is not None:
            i=self.carry
            held_q=q if arm==i else self.q[i]
            out['broom']=b.fk(held_q,self.robots[i]['base_np'])@np.linalg.inv(self.ee[i])
        return out
    def hits(self,arm,q,grip=None,ignore_other=False):
        if np.any(np.abs(q[:5])>10) or abs(q[5])>3.14:
            return [(self.robots[arm]['prefix']+'_joint_limit','URDF_limit')]
        mats=self.matrices(arm,q,grip)
        prefix=self.robots[arm]['prefix']+'_'
        hits=[]
        for a,c in self.world.pairs:
            if not(a.startswith(prefix) or c.startswith(prefix) or (self.carry==arm and c=='broom')):continue
            if ignore_other and a.startswith(('Left_','Right_')) and c.startswith(('Left_','Right_')) and a.split('_')[0]!=c.split('_')[0]:continue
            if intersects(self.world.geometry[a],mats[a],self.world.geometry[c],mats[c]):hits.append((a,c))
        return hits
    def solve(self,arm,target,seed,best_of=False):
        robot=self.robots[arm]
        candidates=[seed]
        label=f'{self.frame}_{arm}'
        if label in self.seeds:candidates.append(np.array(self.seeds[label]))
        candidates.extend([np.zeros(6),np.array([0,-1.3,1.6,0,0,0]),np.array([0,1.3,-1.6,0,0,0])])
        candidates.extend(np.random.default_rng(123+arm).uniform(-math.pi,math.pi,(45,6)))
        rejected=[];solutions=[]
        for k,s in enumerate(candidates):
            # One numerical seed at a time; collision filtering is authoritative.
            q,err=single_ik(target,robot,s)
            if err[0]>.002 or err[1]>.015:continue
            q=seed+(q-seed+math.pi)%(2*math.pi)-math.pi
            q[5]=(q[5]+math.pi)%(2*math.pi)-math.pi
            hits=self.hits(arm,q)
            if hits:
                if len(rejected)<4:rejected.append(hits)
                continue
            if best_of:
                if not any(np.linalg.norm(q-s[0])<.05 for s in solutions):solutions.append((q,err))
                continue
            if k>0:
                self.restarts+=1;self.saved[label]=q.tolist()
                print('BRANCH',self.frame,arm,k,flush=True)
            return q,err
        if solutions:
            solutions.sort(key=lambda item:np.linalg.norm(item[0]-seed)+(5 if item[0][2]<-.1 else 0))
            print('GOALS',self.frame,arm,[np.round(item[0],3).tolist() for item in solutions],flush=True)
            self.saved[label]=solutions[0][0].tolist()
            return solutions[0]
        (OUT/'planning_failure.json').write_text(json.dumps({'frame':self.frame,'arm':arm,'rejected_pairs':rejected,'target':target.tolist()},indent=2))
        raise RuntimeError(f'No collision-free IK at {self.frame}, arm {arm}; {rejected}')
    def calibrate(self,arm,grasp,tool_name='broom',bias=.145):
        ee=grasp@b.np_transform((-bias,0,0))
        tool=self.world.geometry[tool_name]
        # Search within the URDF's legal 0..44mm travel. Keep 0.4mm extra gap.
        for opening in np.arange(0,.04401,.00025):
            hits=[]
            for j in b.GRIPPER_JOINTS:
                mesh=self.world.geometry[self.robots[arm]['prefix']+'_'+j.child]
                m=ee@b.np_transform(np.array(j.xyz)+np.array(j.axis)*opening)
                hits.append(intersects(mesh,m,tool,np.eye(4)))
            if not any(hits):return float(opening+.0004)
        raise RuntimeError('No legal nonpenetrating gripper aperture')

    def transfer(self,arm,goal):
        start=self.q[arm].copy()
        print('TRANSFER',self.frame,arm,'start_hits',self.hits(arm,start),'goal_hits',self.hits(arm,goal),flush=True)
        def valid(q):
            if self.hits(arm,q):return False
            if self.carry==arm:
                mats=self.matrices(arm,q)
                if arm==1 and self.world.geometry['broom'].bounds(mats['broom'])[0][2]<.812:
                    return False
                for obstacle in ('Table','broom_shovel'):
                    if intersects(self.world.geometry['broom'],mats['broom'],self.world.geometry[obstacle],mats[obstacle]):return False
            return True
        def edge(a,c):
            n=max(1,int(np.ceil(np.max(np.abs(c-a))/.035)))
            return all(valid(a+(c-a)*k/n) for k in range(1,n+1))
        if not valid(goal):raise RuntimeError(f'Transfer goal collides at {self.frame}')
        if edge(start,goal):return [start,goal]
        # Search joint-order detours before stochastic search. These are useful
        # near the folded home pose, where simultaneous joint changes clip the
        # table even though lifting the shoulder first is safe.
        states={0:(start,None)};dead=set()
        def ordered(mask):
            if mask==63:
                route=[];cursor=mask
                while cursor is not None:
                    route.append(states[cursor][0]);cursor=states[cursor][1]
                return route[::-1]
            for j in (1,3,2,0,4,5):
                nxt=mask|(1<<j)
                if nxt==mask or nxt in dead:continue
                q=states[mask][0].copy();q[j]=goal[j]
                if edge(states[mask][0],q):
                    states[nxt]=(q,mask)
                    route=ordered(nxt)
                    if route is not None:return route
            dead.add(mask)
            return None
        route=ordered(0)
        if route:
            print('ORDERED_PATH',self.frame,arm,len(route),flush=True)
            return route
        rng=np.random.default_rng(800+self.frame+arm)
        trees=[([start],[-1]),([goal],[-1])]
        def extend(tree,target):
            nodes,parents=tree
            idx=int(np.argmin([np.linalg.norm(n-target) for n in nodes]));a=nodes[idx]
            distance=np.linalg.norm(target-a);c=a+(target-a)*min(1,.35/max(distance,1e-9))
            if not edge(a,c):return None,False
            nodes.append(c);parents.append(idx)
            return len(nodes)-1,distance<=.35
        def chain(tree,i):
            nodes,parents=tree;out=[]
            while i>=0:out.append(nodes[i]);i=parents[i]
            return out[::-1]
        for iteration in range(900):
            active=iteration%2;other=1-active
            sample=trees[other][0][-1] if rng.random()<.2 else ((start+goal)/2+rng.normal(0,1.15,6))
            ia,_=extend(trees[active],sample)
            if ia is None:continue
            target=trees[active][0][ia]
            for _ in range(70):
                ib,reached=extend(trees[other],target)
                if ib is None:break
                if reached:
                    p=chain(trees[active],ia);q=chain(trees[other],ib)
                    path=p+q[-2::-1] if active==0 else q+p[-2::-1]
                    for _ in range(60):
                        if len(path)<3:break
                        i,j=sorted(rng.choice(len(path),2,replace=False))
                        if j>i+1 and edge(path[i],path[j]):path=path[:i+1]+path[j:]
                    print('RRT',self.frame,arm,'nodes',sum(len(t[0]) for t in trees),'path',len(path),flush=True)
                    return path
            if iteration%100==0:print('RRT_SEARCH',self.frame,arm,iteration,[len(t[0]) for t in trees],flush=True)
        raise RuntimeError(f'RRT did not connect frame {self.frame}, arm {arm}')

def path_at(path,t):
    distances=[np.linalg.norm(c-a) for a,c in zip(path,path[1:])]
    position=np.clip(t,0,1)*sum(distances)
    for a,c,length in zip(path,path[1:],distances):
        if position<=length:return a+(c-a)*(position/max(length,1e-12))
        position-=length
    return path[-1]

def single_ik(target,robot,seed):
    q=np.array(seed,float).copy()
    for _ in range(180):
        cur=b.fk(q,robot['base_np'])
        e=np.r_[target[:3,3]-cur[:3,3],.18*v.rotvec(target[:3,:3]@cur[:3,:3].T)]
        if np.linalg.norm(e)<.00008:break
        j=np.zeros((6,6))
        for i in range(6):
            qs=q.copy();qs[i]+=.0001;f=b.fk(qs,robot['base_np'])
            j[:,i]=np.r_[(f[:3,3]-cur[:3,3])/.0001,.18*v.rotvec(f[:3,:3]@cur[:3,:3].T)/.0001]
        dq=j.T@np.linalg.solve(j@j.T+np.eye(6)*.000016,e)
        q+=dq*min(1,.2/max(np.linalg.norm(dq),1e-9))
    f=b.fk(q,robot['base_np'])
    return q,(float(np.linalg.norm(target[:3,3]-f[:3,3])),float(np.linalg.norm(v.rotvec(target[:3,:3]@f[:3,:3].T))))

def search_handoff(planner,ee):
    choices=[];counter=0
    seeds=[np.zeros(6),np.array([0,1.5,1.8,-1,0,0]),np.array([0,1.5,-1.8,-1,0,0]),np.array([-1.5,1.5,1.8,-1,1,1])]
    for z in (.98,1.08,1.18):
        for y in (-.28,-.16):
            for yaw in np.arange(0,2*math.pi,math.pi/3):
                for tilt in (-math.pi/2,math.pi/2):
                    tool=v.pose((0,y,z),b.np_rpy((0,0,yaw))@b.np_rpy((tilt,0,0)))
                    planner.tool=tool;planner.grips=[.03,.03];qs=[];ok=True
                    for arm in (1,0):
                        target=tool@ee[arm]@(b.np_transform((-.055,0,0)) if arm==1 else np.eye(4))
                        found=[]
                        for seed in seeds:
                            q,err=single_ik(target,planner.robots[arm],seed)
                            q=(q+math.pi)%(2*math.pi)-math.pi
                            if err[0]<.001 and err[1]<.01 and not planner.hits(arm,q,ignore_other=True):found.append(q)
                        if not found:ok=False;break
                        found.sort(key=lambda q:np.linalg.norm(q)+(4 if q[2]<0 else 0))
                        qs.append(found[0])
                    if ok:
                        planner.q=[qs[1],qs[0]]
                        if not planner.hits(0,qs[1]) and not planner.hits(1,qs[0]):
                            score=sum(np.linalg.norm(q) for q in qs)+sum(4 for q in qs if q[2]<0)
                            choices.append({'score':float(score),'tool':tool.tolist(),'approach_q':[q.tolist() for q in planner.q]})
                            print('HANDOFF_CANDIDATE',counter,round(score,2),[round(float(yaw),2),round(float(tilt),2),y,z],flush=True)
                    planner.q=[np.zeros(6),np.zeros(6)];counter+=1
                    if counter%12==0:print('HANDOFF_SEARCH',counter,flush=True)
    choices.sort(key=lambda item:item['score'])
    (OUT/'handoff_candidates.json').write_text(json.dumps(choices,indent=2))
    if not choices:raise RuntimeError('No collision-free handoff configuration found')
    (OUT/'handoff_plan.json').write_text(json.dumps(choices[0],indent=2))
    print('BEST_HANDOFF',json.dumps(choices[0]),flush=True)

def main():
    # Use the existing builder for asset/physics creation, instrument its small
    # functions to capture scene objects, then supply collision-aware IK.
    captured={};original_objects=b.create_official_objects;original_robot=b.create_robot
    def create_robot(*args,**kw):
        r=original_robot(*args,**kw);captured.setdefault('robots',[]).append(r);return r
    def create_objects(*args,**kw):
        o=original_objects(*args,**kw);captured['objects']=o;return o
    b.create_robot=create_robot;b.create_official_objects=create_objects
    # The new builder extension performs the trajectory planning below.
    build(captured)

def build(captured):
    # Implementation is separate from the earlier verified artifact.
    bpy.ops.wm.read_factory_settings(use_empty=True);b.linear_keyframes()
    mats={k:b.make_material(k,c) for k,c in {
        'black':(.035,.045,.06,1),'white':(.65,.72,.78,1),'table':(.20,.25,.28,1),
        'table_edge':(.08,.11,.14,1),'ground':(.04,.055,.075,1),'proxy':(1,0,1,1),
        'cube_red':(.8,.06,.03,1),'cube_green':(.04,.55,.15,1),'cube_blue':(.03,.18,.8,1)}.items()}
    b.setup_scene(mats);scene=bpy.context.scene;scene.render.fps=25
    scene.camera.location=(1.25,1.35,1.95)
    scene.camera.rotation_euler=(Vector((0,-.05,.91))-scene.camera.location).to_track_quat('-Z','Y').to_euler()
    scene.camera.data.lens=60
    for light in bpy.data.lights:light.energy*=.2
    scene.render.resolution_x=960;scene.render.resolution_y=720
    robots=[b.create_robot('Left',(-.3,-.45,.765),mats),b.create_robot('Right',(.3,-.45,.765),mats)]
    objects=b.create_official_objects(mats)
    pan=objects['pan'];b.add_rigid_body(pan,'PASSIVE','MESH',friction=.45);pan.rigid_body.collision_margin=.0001
    objects['broom_visual'].location=(0,0,0)
    collider=objects['broom_collider'];collider.scale=(.019/.016,.027/.028,.053/.018)
    bpy.context.view_layer.objects.active=collider;collider.select_set(True)
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);collider.select_set(False)
    collider.rigid_body.collision_margin=.0001
    for cube in objects['cubes']:
        cube.rigid_body.friction=.3;cube.rigid_body.linear_damping=.15;cube.rigid_body.angular_damping=.2
    b.configure_physics();scene.rigidbody_world.substeps_per_frame=24;scene.rigidbody_world.solver_iterations=50
    bpy.context.view_layer.update()
    planner=Planner(robots,objects)
    pan_t=np.array(pan.matrix_world)
    initial=v.pose(objects['broom_initial'],objects['broom_initial_quat'].to_matrix())
    grasp=[v.pose((0,.013,-.035),np.diag([-1.,-1.,1.])),v.pose((0,.022,-.07),[[0,1,0],[-.5,0,math.sqrt(.75)],[math.sqrt(.75),0,.5]])]
    biases=[.155,.145]
    openings=[planner.calibrate(i,g,bias=biases[i]) for i,g in enumerate(grasp)]
    ee=[g@b.np_transform((-biases[i],0,0)) for i,g in enumerate(grasp)]
    planner.ee=ee
    if '--search-handoff' in sys.argv:
        search_handoff(planner,ee)
        return
    head=b.np_transform((0,-.014,.071))
    handoff=v.pose((0,-.18,1.04),[[-1,0,0],[0,0,1],[0,1,0]])
    def raised(t,h=.055):
        t=t.copy();t[2,3]+=h;return t
    def retract(t,d=.055):return t@b.np_transform((-d,0,0))
    def world(p):return (pan_t@np.r_[p,1])[:3]
    # Hold the pan from the handle end, keeping the bulky wrist behind it and
    # out of the right arm's sweep corridor.
    pan_tcp=pan_t@v.pose((0,.105,.018),[[0,1,0],[-math.sqrt(.75),0,-.5],[-.5,0,math.sqrt(.75)]])
    pan_grasp=pan_tcp@b.np_transform((-.145,0,0))
    planner.tool=b.np_transform((100,100,100))
    left_pan_q,_=planner.solve(0,pan_grasp,np.zeros(6),best_of=True)
    planner.q[0]=left_pan_q
    def heading_for(contact):
        radial=np.asarray(contact)-np.array([.3,-.45,.792]);radial[2]=0
        radius=np.linalg.norm(radial);radial/=radius
        wrist_offset=.071+.07+.145*math.sqrt(.75)
        cosine=np.clip((radius*radius+wrist_offset*wrist_offset-.24*.24)/(2*radius*wrist_offset),-1,1)
        angle=math.acos(cosine)
        return np.array([math.cos(angle)*radial[0]-math.sin(angle)*radial[1],math.sin(angle)*radial[0]+math.cos(angle)*radial[1],0])
    def brush(contact,d,heading=None):
        # Keep the wrist on the base side of the brush, rather than forcing
        # an infeasible wrist-down pose. The head can push with any edge.
        if heading is None:heading=heading_for(contact)
        r=np.column_stack(([-heading[1],heading[0],0],[0,0,1],heading))
        return v.pose(contact-r@head[:3,3],r)
    def stroke(start,end,d,label):
        candidates=[]
        for h in (d,-d,np.array([-d[1],d[0],0]),np.array([d[1],-d[0],0])):
            across=np.array([-h[1],h[0],0])
            extent=.053*abs(np.dot(h,d))+.019*abs(np.dot(across,d))
            a=brush(start-d*(extent+.0175+.014),d,h)
            c=brush(end-d*(extent+.0175-.003),d,h)
            poses=[a,v.interp(a,c,.5),c];qs=[];okay=True
            for tool in poses:
                planner.tool=tool;planner.grips=[.03,openings[1]]
                target=tool@ee[1];found=None
                seeds=([qs[-1]] if qs else [])+[np.zeros(6),np.array([0,1.5,1.8,-1,0,0]),np.array([-1.5,1.5,1.8,-1,1,1]),np.array([1,1,1.5,0,1,0])]
                for seed in seeds:
                    q,err=single_ik(target,robots[1],seed);q=(q+math.pi)%(2*math.pi)-math.pi
                    if err[0]<.0015 and err[1]<.015 and q[2]>0 and not planner.hits(1,q):found=q;break
                if found is None:okay=False;break
                qs.append(found)
            if okay:candidates.append((sum(np.linalg.norm(q) for q in qs),a,c,h))
        if not candidates:raise RuntimeError(f'No feasible face-aligned stroke: {label}')
        _,a,c,h=min(candidates,key=lambda item:item[0])
        print('STROKE',label,'heading',h.tolist(),flush=True)
        return a,c
    def offset(center,d,extra):
        heading=heading_for(center)
        extent=.053*abs(np.dot(heading,d))+.019*abs(np.dot(np.array([-heading[1],heading[0],0]),d))
        return center-d*(extent+.0175+extra)
    keys=[(1,initial),(65,initial),(115,raised(initial,.04)),(180,handoff),(215,handoff)]
    feedback_path=OUT/'sweep_feedback.json'
    feedback=json.loads(feedback_path.read_text()) if feedback_path.exists() else None
    for i,(cfg,lane) in enumerate(zip(objects['layout']['Rigid']['small_cube'],[-.05,0,.05])):
        start=np.array(cfg['default_pos']);start[2]=.792
        stage=world((lane,-.215,0));stage[2]=.792
        entry_lift=.008 if i==2 else .016
        final=world(([-.025,.025,0][i],[-.032,-.032,-.096][i],0));final[2]=.7955+entry_lift
        d=stage-start;d[2]=0;d/=np.linalg.norm(d)
        p0,p1=stroke(start,stage,d,f'{i}_stage')
        measured=np.array(feedback['staging_positions'][i]) if feedback else stage.copy();measured[2]=.792+entry_lift
        inward=final-measured;inward[2]=0;inward/=np.linalg.norm(inward)
        p2,p3=stroke(measured,final,inward,f'{i}_pan')
        t=240+i*205
        keys.extend([(t,raised(p0)),(t+20,p0),(t+65,p1),(t+80,raised(p1)),(t+110,raised(p2)),(t+125,p2),(t+165,p3),(t+180,raised(p3))])
    park=brush(np.array([.36,.12,.792]),np.array([1.,0,0]))
    keys.extend([(890,raised(park,.08)),(925,park),(1000,park)])
    homes=[b.fk(np.zeros(6),r['base_np']) for r in robots]
    # Same calibrated handle-end grasp used while screening sweep poses.
    pan_local=np.linalg.inv(pan_t)@pan_tcp
    pan_open=planner.calibrate(0,pan_local,'broom_shovel')
    calibration={'grasp_matrices':[g.tolist() for g in grasp],'tool_to_flange':[g.tolist() for g in ee],'tcp_biases':biases,'openings':openings,'pan_opening':pan_open}
    (OUT/'grasp_calibration.json').write_text(json.dumps(calibration,indent=2))
    print('CALIBRATED',calibration,flush=True)
    planner.q=[np.zeros(6),np.zeros(6)]
    qtrace=[];errors=[];jumps=[];transfers={}
    free_ranges={0:[(1,35),(115,180),(225,250),(900,1000)],1:[(95,125),(215,240),(320,350),(420,445),(525,555),(625,650),(730,760),(830,890),(960,1000)]}
    try:
        for f in range(1,1001):
            tool=v.sample(keys,f);planner.tool=tool;planner.frame=f
            planner.carry=0 if 65<=f<190 else (1 if 190<=f<=925 else None)
            left_keys=[(1,homes[0]),(35,retract(initial@ee[0])),(55,initial@ee[0]),(65,initial@ee[0])]
            if f<=65:lt=v.sample(left_keys,f)
            elif f<=190:lt=tool@ee[0]
            elif f<=205:lt=handoff@ee[0]
            elif f<=225:lt=v.interp(handoff@ee[0],retract(handoff@ee[0]),(f-205)/20)
            elif f<=250:lt=v.interp(retract(handoff@ee[0]),retract(pan_grasp),(f-225)/25)
            elif f<=270:lt=v.interp(retract(pan_grasp),pan_grasp,(f-250)/20)
            elif f<=880:lt=pan_grasp
            elif f<=900:lt=v.interp(pan_grasp,retract(pan_grasp),(f-880)/20)
            else:lt=v.interp(retract(pan_grasp),homes[0],min(1,(f-900)/90))
            if f<=95:rt=homes[1]
            elif f<=125:rt=v.interp(homes[1],retract(handoff@ee[1]),(f-95)/30)
            elif f<=170:rt=retract(handoff@ee[1])
            elif f<=190:rt=v.interp(retract(handoff@ee[1]),handoff@ee[1],(f-170)/20)
            elif f<=925:rt=tool@ee[1]
            elif f<=940:rt=park@ee[1]
            elif f<=960:rt=v.interp(park@ee[1],retract(park@ee[1]),(f-940)/20)
            else:rt=v.interp(retract(park@ee[1]),homes[1],min(1,(f-960)/40))
            def lerp(a,c,t):return a+(c-a)*np.clip(t,0,1)
            lg=.03 if f<50 else (lerp(.03,openings[0],(f-50)/15) if f<65 else openings[0])
            if f>190:lg=lerp(openings[0],.03,(f-190)/15)
            if 255<=f<=880:lg=lerp(.03,pan_open,(f-255)/15)
            if 880<f:lg=lerp(pan_open,.03,(f-880)/20)
            rg=.03 if f<175 else lerp(.03,openings[1],(f-175)/15)
            if f>925:rg=lerp(openings[1],.03,(f-925)/15)
            planner.grips=[float(lg),float(rg)]
            for i,target in enumerate((lt,rt)):
                old=planner.q[i].copy()
                free=next(((start,end) for start,end in free_ranges[i] if start<f<=end),None)
                if free:
                    start,end=free;key=(i,start)
                    if key not in transfers:
                        if end==1000:goal=np.zeros(6)
                        else:
                            if i==0:goal_target=retract(initial@ee[0]) if end==35 else (handoff@ee[0] if end==180 else retract(pan_grasp))
                            else:goal_target=retract(handoff@ee[1]) if end==125 else v.sample(keys,end)@ee[1]
                            goal,_=planner.solve(i,goal_target,old,best_of=True)
                        transfers[key]=(start,planner.transfer(i,goal))
                    route_start,path=transfers[key]
                    q=path_at(path,(f-route_start)/(end-route_start));err=(0.,0.)
                    if planner.hits(i,q):
                        path=planner.transfer(i,path[-1]);route_start=f-1
                        transfers[key]=(route_start,path);q=path_at(path,1/(end-route_start))
                        if planner.hits(i,q):raise RuntimeError(f'Dynamic transfer conflict at {f}')
                else:q,err=planner.solve(i,target,old)
                delta=float(np.max(np.abs(q-old)))
                if delta>.20:jumps.append({'frame':f,'arm':i,'max_joint_delta':delta})
                planner.q[i]=q;errors.append(err)
                b.set_joint_pose(robots[i],q,planner.grips[i],f)
            owner=0 if 65<=f<190 else (1 if 190<=f<=925 else None)
            actual_tool=tool if owner is None else b.fk(planner.q[owner],robots[owner]['base_np'])@np.linalg.inv(ee[owner])
            v.keymat(objects['broom_driver'],f,actual_tool);v.keymat(collider,f,actual_tool@head)
            qtrace.append({'frame':f,'q':[q.tolist() for q in planner.q],'grips':planner.grips})
            if f%50==0:print('PLANNED',f,flush=True)
    finally:
        planner.seeds.update(planner.saved)
        planner.warm.write_text(json.dumps(planner.seeds,indent=2))
        (OUT/'joint_trace.json').write_text(json.dumps(qtrace))
        (OUT/'joint_jumps.json').write_text(json.dumps(jumps,indent=2))
        # Preserve every turn of a searched path, even if it falls between
        # integer frames; otherwise animation interpolation cuts the corner.
        for (arm,begin),(route_start,path) in transfers.items():
            end=next(end for start,end in free_ranges[arm] if start==begin)
            lengths=[np.linalg.norm(c-a) for a,c in zip(path,path[1:])]
            total=sum(lengths);distance=0.
            for index,node in enumerate(path[:-1]):
                when=route_start+(end-route_start)*distance/max(total,1e-12)
                if index>0 and when<end and when<=len(qtrace):
                    grip=qtrace[max(0,min(len(qtrace)-1,int(when)-1))]['grips'][arm]
                    b.set_joint_pose(robots[arm],node,grip,float(when))
                distance+=lengths[index]
        # Constraints, not independently interpolated tool keys, enforce the
        # grasp at fractional frames as well as at integer physics steps.
        driver=objects['broom_driver']
        for i,robot in enumerate(robots):
            anchor=bpy.data.objects.new(f'{robot["prefix"]}_tool_anchor',None)
            bpy.context.collection.objects.link(anchor)
            anchor.parent=robot['frames']['link6']
            anchor.matrix_basis=Matrix(np.linalg.inv(ee[i]).tolist())
            constraint=driver.constraints.new('COPY_TRANSFORMS');constraint.name=f'Grasp_{i}'
            constraint.target=anchor
            timeline=[(1,0),(64,0),(65,1),(189,1),(190,0),(1000,0)] if i==0 else [(1,0),(189,0),(190,1),(925,1),(926,0),(1000,0)]
            for frame,value in timeline:
                constraint.influence=value;constraint.keyframe_insert('influence',frame=frame)
        # Constant ownership transitions; pose keys themselves remain linear.
        if driver.animation_data and driver.animation_data.action:
            for layer in driver.animation_data.action.layers:
                for strip in layer.strips:
                    for bag in strip.channelbags:
                        for curve in bag.fcurves:
                            if 'constraints[' in curve.data_path:
                                for key in curve.keyframe_points:key.interpolation='CONSTANT'
        collider.animation_data_clear();collider.parent=driver
        collider.matrix_basis=Matrix(head.tolist())
        bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'sweep_blocks_collision.blend'))
    print('PLANNING_DONE',len(jumps),'jumps',flush=True)

if __name__=='__main__':main()
