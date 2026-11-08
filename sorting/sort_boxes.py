#!/usr/bin/env python3
"""Simulation-only fixed-bin sorting with explicit Gazebo attachment grasps."""
import sys,math,json,time,fcntl
from itertools import product
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene,CollisionObject,AttachedCollisionObject,ObjectColor,AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene,GetPositionFK,GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.stringmsg_pb2 import StringMsg
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'reinforcement_learning'))
from reach_env import ReachEnv,NAMES
sys.path.insert(0,str(ROOT/'model'))
from geometry import L1,L2,SHOULDER_HEIGHT,TOOL_OFFSET,REVISION,GEOMETRY
LAYOUT=json.loads((Path(__file__).parent/'layout.json').read_text())


def joints(angle,z,radius=.30):
    """Analytic tilted-grasp IK for the measured two-link lengths and provisional offsets."""
    if GEOMETRY.get("joint_axes")==["z","y","y","z","y"]:
        raise RuntimeError("Sorting IK requires updating for M4 roll/M5 pitch; use no-box commissioning instead.")
    # Long wrist/tool requires varying pitch between ground pickup and raised transit.
    preferred=2.5-0.5*max(0.,min(1.,(z-.01)/.11))
    candidates=sorted(np.linspace(1.7,2.7,201),key=lambda p:abs(p-preferred))
    for pitch in candidates:
        radial=radius-TOOL_OFFSET*math.sin(pitch)
        height=z-TOOL_OFFSET*math.cos(pitch)-SHOULDER_HEIGHT
        c=(radial**2+height**2-L1**2-L2**2)/(2*L1*L2)
        if abs(c)>1:continue
        elbow=math.acos(c)
        shoulder=math.atan2(radial,height)-math.atan2(L2*math.sin(elbow),L1+L2*math.cos(elbow))
        q=[angle,shoulder,elbow,pitch-shoulder-elbow,0.]
        if all(lo<=v<=hi for v,lo,hi in zip(q,[-2.8,-1.4,-2,-1.5,-2.8],[2.8,1.4,2,1.5,2.8])):
            return q
    raise ValueError('Unreachable pose within current joint limits')



def inside_bin(xyz,quat,bin):
    """Require the entire 20 mm cube to fit inside the bin, not just its centre."""
    corners=Rotation.from_quat(quat).apply(np.array(list(product([-.01,.01],repeat=3))))+xyz
    centre=np.array(bin['centre'])
    return bool(np.all(np.abs(corners[:,:2]-centre[:2])<bin['inner_half_width']-.001)
                and np.min(corners[:,2])>=.004 and np.max(corners[:,2])<=.045
                and abs(xyz[2]-.015)<.004)


def pose(xyz,quat=(0.,0.,0.,1.)):
    p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
    return p


def box(name,xyz,size,quat=(0.,0.,0.,1.)):
    obj=CollisionObject();obj.id=name;obj.header.frame_id='world'
    obj.operation=CollisionObject.ADD
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(size))]
    obj.primitive_poses=[pose(xyz,quat)]
    return obj


class Sorter(ReachEnv):
    def __init__(self):
        super().__init__()
        self.gz=GzNode();self.poses={};self.states={};self.held=None
        self.gz.subscribe(Pose_V,'/world/arm_lab/dynamic_pose/info',self.on_poses)
        self.pubs={}
        for color in LAYOUT['objects']:
            self.gz.subscribe(StringMsg,f'/sorting/{color}/state',lambda msg,c=color:self.states.update({c:msg.data}))
            for action in ['attach','detach']:
                self.pubs[color,action]=self.gz.advertise(f'/sorting/{color}/{action}',Empty)
        self.scene=self.node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.grip=ActionClient(self.node,FollowJointTrajectory,'/sim_gripper_controller/follow_joint_trajectory')
        if not self.scene.wait_for_service(timeout_sec=10) or not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError('Simulation planning scene/gripper unavailable')
        self.wait_until(lambda:all(n in self.poses for n in LAYOUT['objects']),10,'Box poses unavailable')

    def wait_until(self,predicate,timeout,message):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:raise RuntimeError(message)
            rclpy.spin_once(self.node,timeout_sec=.05)

    def on_poses(self,msg):
        for item in msg.pose:
            if item.name in LAYOUT['objects']:
                self.poses[item.name]=(np.array([item.position.x,item.position.y,item.position.z]),
                                      np.array([item.orientation.x,item.orientation.y,item.orientation.z,item.orientation.w]),time.monotonic())

    def object_pose(self,name):
        if name not in self.poses or time.monotonic()-self.poses[name][2]>2:
            raise RuntimeError(f'Stale Gazebo pose: {name}')
        return self.poses[name][:2]

    def attachment(self,color,attach):
        action='attach' if attach else 'detach';expected='attached' if attach else 'detached'
        # Publish repeatedly to cover transport discovery. State feedback confirms the change.
        for _ in range(30):
            self.pubs[color,action].publish(Empty())
            until=time.monotonic()+.1
            while time.monotonic()<until:rclpy.spin_once(self.node,timeout_sec=.02)
            if self.states.get(color)==expected:return
        raise RuntimeError(f'No Gazebo attachment confirmation for {color}; launch with sorting:=true')

    def apply(self,scene):
        scene.is_diff=True;scene.robot_state.is_diff=True
        reply=self._wait(self.scene.call_async(ApplyPlanningScene.Request(scene=scene)),10)
        if not reply.success:raise RuntimeError('MoveIt scene update failed')

    def support_contact(self,name,allowed):
        query=self.node.create_client(GetPlanningScene,'/get_planning_scene')
        if not query.wait_for_service(timeout_sec=5):raise RuntimeError('Planning scene query unavailable')
        req=GetPlanningScene.Request();req.components.components=128
        matrix=self._wait(query.call_async(req),5).scene.allowed_collision_matrix
        support=LAYOUT['objects'][name]['support']
        for item in [name,support]:
            if item not in matrix.entry_names:
                matrix.entry_names.append(item)
                for row in matrix.entry_values:row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        i=matrix.entry_names.index(name);j=matrix.entry_names.index(support)
        matrix.entry_values[i].enabled[j]=allowed;matrix.entry_values[j].enabled[i]=allowed
        scene=PlanningScene();scene.allowed_collision_matrix=matrix;self.apply(scene)
        self.node.destroy_client(query)

    def sync_scene(self):
        scene=PlanningScene()
        scene.world.collision_objects.append(box('sorting_ground',[0.,0.,-.012],[2.,2.,.02]))
        for part in LAYOUT['static']:
            scene.world.collision_objects.append(box(part['name'],part['xyz'],part['size']))
            color=ObjectColor(id=part['name']);color.color.r,color.color.g,color.color.b,color.color.a=part['color']
            scene.object_colors.append(color)
        for name in LAYOUT['objects']:
            if name==self.held:continue
            xyz,quat=self.object_pose(name)
            scene.world.collision_objects.append(box(name,xyz,[.02]*3,quat))
            c=ObjectColor(id=name);shade=.03 if name.startswith('black') else .95
            c.color.r=c.color.g=c.color.b=shade;c.color.a=1.;scene.object_colors.append(c)
        self.apply(scene)

    def attach_scene(self,name):
        req=GetPositionFK.Request();req.header.frame_id='world';req.fk_link_names=['gripper_palm']
        req.robot_state.joint_state.name=NAMES;req.robot_state.joint_state.position=list(self.q)
        p=self._wait(self.fk.call_async(req),5).pose_stamped[0].pose
        inv=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).inv()
        xyz,quat=self.object_pose(name)
        relative=inv.apply(xyz-np.array([p.position.x,p.position.y,p.position.z]))
        relative_q=(inv*Rotation.from_quat(quat)).as_quat()
        obj=box(name,relative,[.02]*3,relative_q);obj.header.frame_id='gripper_palm'
        attached=AttachedCollisionObject(link_name='gripper_palm',object=obj,
                     touch_links=['gripper_palm','left_finger','right_finger','camera_link','wrist_pitch'])
        # MoveIt automatically removes the world object when attaching it.
        scene=PlanningScene();scene.robot_state.attached_collision_objects=[attached]
        self.apply(scene);self.held=name

    def detach_scene(self,name):
        scene=PlanningScene();attached=AttachedCollisionObject(link_name='gripper_palm')
        attached.object.id=name;attached.object.operation=CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects=[attached]
        self.apply(scene);self.held=None

    def move_to(self,q,label):
        print(label,flush=True);self.sync_scene()
        if not self._move(q):raise RuntimeError(f'Planning failed: {label}')
        if self.held:
            xyz,_=self.object_pose(self.held)
            if np.linalg.norm(xyz-self._position(self.q))>.018:
                raise RuntimeError('Box did not follow gripper; stopping')

    def gripper(self,value):
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['sim_finger_joint']
        point=JointTrajectoryPoint(positions=[value]);point.time_from_start.sec=2
        goal.trajectory.points=[point]
        handle=self._wait(self.grip.send_goal_async(goal),5)
        if not handle.accepted:raise RuntimeError('Gripper rejected motion')
        try:result=self._wait(handle.get_result_async(),8).result
        except (RuntimeError,KeyboardInterrupt):
            self._wait(handle.cancel_goal_async(),5);raise
        if result.error_code!=0:raise RuntimeError('Gripper movement failed')

    def run(self):
        print('SIMULATION: known box colours, planned motions, attachment-based grasps.',flush=True)
        for color in LAYOUT['objects']:
            # Establish a known state even after an interrupted previous run.
            deadline=time.monotonic()+1.0
            while time.monotonic()<deadline:
                self.pubs[color,'detach'].publish(Empty())
                rclpy.spin_once(self.node,timeout_sec=.05)
            self.attachment(color,True)
            self.attachment(color,False)
        query=self.node.create_client(GetPlanningScene,'/get_planning_scene')
        if not query.wait_for_service(timeout_sec=5):raise RuntimeError('Planning scene query unavailable')
        request=GetPlanningScene.Request();request.components.components=4
        existing=self._wait(query.call_async(request),5).scene.robot_state.attached_collision_objects
        for obj in existing:
            if obj.object.id in LAYOUT['objects']:self.detach_scene(obj.object.id)
        self.gripper(.30)
        self.sync_scene()
        results=[]
        placed={'black':0,'white':0}
        for name,item in LAYOUT['objects'].items():
            color=item['color'];bin=LAYOUT['bins'][color];xyz,_=self.object_pose(name)
            source=np.array(item['xyz'])
            drop=bin['slots'][placed[color]]
            drop_angle=math.atan2(drop[1],drop[0]);drop_radius=math.hypot(drop[0],drop[1])
            if np.linalg.norm(xyz-source)>.012:
                raise RuntimeError(f'{name} is not at its pickup point. Restart the sorting simulation to reset.')
            self.move_to(joints(item['source_angle'],.12,item['source_radius']),f'{name}: approach pickup')
            self.move_to(joints(item['source_angle'],float(source[2]),item['source_radius']),f'{name}: descend')
            if np.linalg.norm(self.object_pose(name)[0]-self._position(self.q))>.008:
                raise RuntimeError('Gripper is not aligned with box')
            self.gripper(.07)
            self.support_contact(name,True)
            self.attachment(name,True);self.attach_scene(name)
            self.move_to(joints(item['source_angle'],.12,item['source_radius']),f'{name}: lift')
            if self.object_pose(name)[0][2]<source[2]+.035:raise RuntimeError('Lift verification failed')
            self.support_contact(name,False)
            self.move_to(joints(drop_angle,.12,drop_radius),f'{name}: carry to fixed bin')
            self.move_to(joints(drop_angle,.07,drop_radius),f'{name}: lower over bin')
            self.gripper(.30)
            self.attachment(name,False);self.detach_scene(name)
            # Wait for gravity and verify a sustained resting position inside the correct bin.
            good_since=None;deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                rclpy.spin_once(self.node,timeout_sec=.05)
                xyz,quat=self.object_pose(name)
                inside=inside_bin(xyz,quat,bin)
                if inside:
                    if good_since is None:good_since=time.monotonic()
                    if time.monotonic()-good_since>.6:break
                else:good_since=None
            else:raise RuntimeError(f'{name} did not settle in the {color} bin')
            placed[color]+=1
            results.append({'box':name,'bin':color,'position_m':xyz.tolist(),'verified':True})
            print(f'VERIFIED: {name} inside {color} bin at {xyz.tolist()}',flush=True)
            self.move_to(joints(drop_angle,.12,drop_radius),f'{name}: retreat')
        self.move_to([0.,.35,-.7,.35,0.],'Return to ready pose')
        report={'geometry_revision':REVISION,'mode':'scripted_attachment_sorting','vision_used':False,'reinforcement_learning_used':False,'seed':LAYOUT['seed'],'results':results}
        (Path(__file__).parent/'last_result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(f'PASS: all {len(results)} boxes verified in their fixed bins.',flush=True)


def main():
    lock=(ROOT/'reinforcement_learning/.training.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Training or another sorting controller is active; stop it first.')
    rclpy.init();env=None
    try:
        env=Sorter();env.run()
    finally:
        if env is not None:env.close()
        rclpy.shutdown()

if __name__=='__main__':main()
