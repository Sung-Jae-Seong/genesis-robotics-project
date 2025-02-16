######## Environment Setting
import os
os.environ['PYOPENGL_PLATFORM'] = 'glx'

import numpy as np
import torch

######## Init Genesis / Generate scene
import genesis as gs
gs.init(backend=gs.cpu)

scene = gs.Scene(
    show_viewer=True,
    sim_options = gs.options.SimOptions(
        dt = 0.01,
        substeps=4,
    ),
)

plane = scene.add_entity(gs.morphs.Plane())
cube = scene.add_entity(gs.morphs.Box(size=(0.03, 0.03, 0.05), pos=(0.7, 0.0, 0.025)))
cube.set_friction(0.1)
m0609 = scene.add_entity(
    gs.morphs.URDF(
        file='/home/sjs/genesis-robotics-project/resource/m0609_gripper2.urdf',
        fixed=True,
    ),
)
m0609.set_friction(0.1)

scene.build()


######## Joint Definition

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

arm_idx = [m0609.get_joint(name).dof_idx_local for name in jnt_names]
gripper_idx = [m0609.get_joint(name).dof_idx_local for name in gripper_names]

m0609.set_dofs_kp(
    kp = np.array([4500*0.7, 4500*0.7, 3500*0.7, 3500*0.7, 2000*0.7, 2000*0.7]),
    dofs_idx_local = arm_idx,
)
m0609.set_dofs_kp(
    kp = np.array([2000, 2000, 2000, 2000]),
    dofs_idx_local = gripper_idx,
)
m0609.set_dofs_kv(
    kv = np.array([500, 500, 450, 450, 400, 400]),
    dofs_idx_local = arm_idx,
)
m0609.set_dofs_kv(
    kv = np.array([1000, 1000, 1000, 1000]),
    dofs_idx_local = gripper_idx,
)
m0609.set_dofs_force_range(
    lower = np.array([-87, -87, -87, -87, -12, -12]),
    upper = np.array([ 87,  87,  87,  87,  12,  12]),
    dofs_idx_local = arm_idx,
)

HAND_OPEN = np.array([0.45, 0.045, -0.45, -0.045])
HAND_CLOSE = np.array([-0.11, -0.011, 0.11, 0.011])

m0609.set_dofs_position(np.array([0, 0, 0, 0, 0, 0]), arm_idx)
m0609.set_dofs_position(np.array(HAND_OPEN), gripper_idx)

end_effector = m0609.get_link('link_6')


#init
qpos = m0609.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.7, 0.0, 0.3]),
    quat = np.array([0, 1, 0, 0]),
)
qpos[6:] = torch.tensor(HAND_OPEN)
path = m0609.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2s duration
)
for waypoint in path:
    m0609.control_dofs_position(
        waypoint[:6],
        arm_idx,
    )
    m0609.control_dofs_position(
        waypoint[6:],
        gripper_idx,
    )
    scene.step()

# reach
qpos = m0609.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.7, 0.0, 0.25]),
    quat = np.array([0, 1, 0, 0]),
)
m0609.control_dofs_position(qpos[:6], arm_idx)
for i in range(100):
    scene.step()
print("reach is done")

# grasp
m0609.control_dofs_position(HAND_CLOSE, gripper_idx)
for i in range(100):
    scene.step()
print("grasp is done")

# lift
qpos = m0609.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.7, 0.0, 0.35]),
    quat=np.array([0, 1, 0, 0]),
)
m0609.control_dofs_position(qpos[:6], arm_idx)
m0609.control_dofs_position(HAND_CLOSE, gripper_idx)

for i in range(200):
    scene.step()
print("lift is done")
