######## Environment Setting
import os
os.environ['PYOPENGL_PLATFORM'] = 'glx' #before import sceneManager

import numpy as np
import torch
from sceneManager import SceneManager
from robotEntity import ManipulatorEntity


######## Init Genesis / Generate scene
import genesis as gs

scene = SceneManager()

plane = scene.add_entity(gs.morphs.Plane())
cube = scene.add_entity(gs.morphs.Box(size=(0.03, 0.03, 0.05), pos=(0.7, 0.0, 0.025)))
cube.set_friction(1.0)
m0609 = scene.add_entity(
    gs.morphs.URDF(
        file='/home/sjs/genesis-robotics-project/resource/m0609_gripper2.urdf',
        fixed=True,
    ),
)
m0609.set_friction(1.0)

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
robot = ManipulatorEntity(m0609, jnt_names, gripper_names)

robot.set_joint_kp(np.array([4500*0.7, 4500*0.7, 3500*0.7, 3500*0.7, 2000*0.7, 2000*0.7]), robot.arm_indices)
robot.set_joint_kp(np.array([2000, 2000, 2000, 2000]), robot.gripper_indices)

robot.set_joint_vel(np.array([500*0.7, 500*0.7, 450*0.7, 450*0.7, 400*0.7, 400*0.7]), robot.arm_indices)
robot.set_joint_vel(np.array([1000, 1000, 1000, 1000]), robot.gripper_indices)

robot.set_position(np.array([0, 0, 0, 0, 0, 0]), robot.arm_indices)

end_effector = robot.model.get_link('link_6')

#init
robot.open_gripper()
qpos = robot.model.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.7, 0.0, 0.3]),
    quat = np.array([0, 1, 0, 0]),
)
qpos[6:] = torch.tensor(robot.open_gripper_joint)
path = robot.model.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2s duration
)
for waypoint in path:
    robot.control_position(waypoint[:6], robot.arm_indices)
    robot.control_position(waypoint[6:], robot.gripper_indices)
    scene.step()

for i in range(100):
    scene.step()

# reach
qpos = robot.model.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.7, 0.0, 0.25]),
    quat = np.array([0, 1, 0, 0]),
)
robot.control_position(qpos[:6], robot.arm_indices)
for i in range(100):
    scene.step()
print("reach is done")

# grasp
robot.close_gripper()
for i in range(100):
    scene.step()
print("grasp is done")

# lift
qpos = robot.model.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.7, 0.0, 0.3]),
    quat=np.array([0, 1, 0, 0]),
)
robot.control_position(qpos[:6], robot.arm_indices)
robot.close_gripper()
for i in range(200):
    scene.step()

import time
time.sleep(1)

print("lift is done")
