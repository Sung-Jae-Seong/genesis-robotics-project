######## Environment Setting

# When : OpenGL.error.Error: Attempt to retrieve context when no valid context
import os
os.environ['PYOPENGL_PLATFORM'] = 'glx' #before import sceneManager

import numpy as np
from sceneManager import SceneManager
from robotEntity import RobotEntity

######## Init Genesis / Generate scene
import genesis as gs

scene = SceneManager()

plane = scene.add_entity(gs.morphs.Plane())
m0609 = scene.add_entity(
    gs.morphs.URDF(
        file='/home/sjs/genesis/resource/m0609_gripper2.urdf',
        fixed=True,
    ),
)

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
    'l_finger_1_joint', # joint_value > 0 : open
    'l_finger_2_joint',
    'r_finger_1_joint', # joint_value < 0 : open
    'r_finger_2_joint',
]
robot = RobotEntity(m0609)

robot.init_arm(jnt_names)
robot.init_hand(gripper_names)

robot.Arm.set_joint_kp(np.array([4500*0.7, 4500*0.7, 3500*0.7, 3500*0.7, 2000*0.7, 2000*0.7]))
robot.Hand.set_joint_kp(np.array([1000, 1000, 1000, 1000]))

robot.Arm.set_joint_vel(np.array([500*0.7, 500*0.7, 450*0.7, 450*0.7, 400*0.7, 400*0.7]))
robot.Hand.set_joint_vel(np.array([500*0.5, 500*0.5, 500*0.5, 500*0.5]))

robot.Arm.set_position(np.array([0, 0, 0, 0, 0, 0]))
robot.Hand.set_position(np.array(robot.Hand.open_joint))


for i in range(750):
    if i == 0:
        robot.Arm.control_position(
            np.array([0, 0, 0, 0, 0, 0]),
        )
        robot.Hand.close()
    elif i == 250:
        robot.Arm.control_position(
            np.array([0, 0, 1.57, 0, 1.57, 0]),
        )
        robot.Hand.open()
    elif i == 500:
        robot.Arm.control_position(
            np.array([0, 0, 0, 0, 0, 0]),
        )
        robot.Hand.close()
    scene.step()

while True:
    scene.step()
    robot.Arm.control_position(
        np.array([0.0, 0, 0, 0, 0, 0]),
    )
    robot.Hand.close()