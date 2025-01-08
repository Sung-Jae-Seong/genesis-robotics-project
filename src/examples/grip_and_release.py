######## Environment Setting
import os
os.environ['PYOPENGL_PLATFORM'] = 'glx' #before import sceneManager

import sys ## For source load
src_path = '/home/sjs/genesis/src'
if src_path not in sys.path:
    sys.path.append(src_path)
if 'PYTHONPATH' in os.environ:  
    os.environ['PYTHONPATH'] += f':{src_path}'
else:
    os.environ['PYTHONPATH'] = src_path

import numpy as np
import torch
from sceneManager import SceneManager
from robotEntity import RobotEntity


######## Init Genesis / Generate scene
import genesis as gs

scene = SceneManager()

plane = scene.add_entity(gs.morphs.Plane())
cube = scene.add_entity(gs.morphs.Box(size=(0.03, 0.03, 0.05), pos=(0.7, 0.0, 0.025)))
cube.set_friction(0.1)
m0609 = scene.add_entity(
    gs.morphs.URDF(
        file='/home/sjs/genesis/resource/m0609_gripper2.urdf',
        fixed=True,
    ),
)
m0609.set_friction(0.1)

scene.build()


########

jnt_names = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
]
gripper_names = [
    'l_finger_1_joint',
    'l_finger_2_joint',
    'r_finger_1_joint',
    'r_finger_2_joint',
]
robot = RobotEntity(m0609)

robot.init_arm(jnt_names)
robot.init_hand(gripper_names)

robot.Arm.set_joint_kp(np.array([4500*0.7, 4500*0.7, 3500*0.7, 3500*0.7, 2000*0.7, 2000*0.7]))
robot.Hand.set_joint_kp(np.array([2000, 2000, 2000, 2000]))

robot.Arm.set_joint_vel(np.array([500*0.7, 500*0.7, 450*0.7, 450*0.7, 400*0.7, 400*0.7]))
robot.Hand.set_joint_vel(np.array([1000, 1000, 1000, 1000]))

robot.Arm.set_position(np.array([0, 0, 0, 0, 0, 0]))
robot.Hand.set_position(np.array(robot.Hand.open_joint))

end_effector = robot.model.get_link('link_6')

#init
robot.Hand.open()
qpos = robot.model.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.7, 0.0, 0.3]),
    quat = np.array([0, 1, 0, 0]),
)
qpos[6:] = torch.tensor(robot.Hand.open_joint)
path = robot.model.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2s duration
)
for waypoint in path:
    robot.Arm.control_position(waypoint[:6])
    robot.Hand.control_position(waypoint[6:])
    scene.step()

for i in range(100):
    scene.step()

# reach
qpos = robot.model.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.7, 0.0, 0.25]),
    quat = np.array([0, 1, 0, 0]),
)
robot.Arm.control_position(qpos[:6])
for i in range(100):
    scene.step()
print("reach is done")

# grasp
robot.Hand.close()
for i in range(100):
    scene.step()
print("grasp is done")

# lift
qpos = robot.model.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.7, 0.0, 0.3]),
    quat=np.array([0, 1, 0, 0]),
)
robot.Arm.control_position(qpos[:6])
robot.Hand.close()
for i in range(200):
    scene.step()
print("lift is done")
