######## Environment Setting
import os
import numpy as np

os.environ['PYOPENGL_PLATFORM'] = 'glx'


######## Init Genesis / Generate scene
import genesis as gs
gs.init(backend=gs.cpu)

scene = gs.Scene(
    show_viewer=True,
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
)

plane = scene.add_entity(gs.morphs.Plane())
#panda = scene.add_entity(gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'))
m0609 = scene.add_entity(
    gs.morphs.URDF(
        file='/home/sjs/genesis/resource/m0609_gripper.urdf',
        fixed=True,
    ),
)

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
dofs_idx = [m0609.get_joint(name).dof_idx_local for name in jnt_names]

gripper_names = [
    'rg2_finger_joint1', #left
    'rg2_finger_joint2', #right
]
gripper_idx = [m0609.get_joint(name).dof_idx_local for name in gripper_names]



######## Joint Init / Setting

# set positional gains
m0609.set_dofs_kp(
    kp             = np.array([4500*0.7, 4500*0.7, 3500*0.7, 3500*0.7, 2000*0.7, 2000*0.7]),
    dofs_idx_local = dofs_idx,
)
# set velocity gains
m0609.set_dofs_kv(
    kv = np.array([500, 500, 450, 450, 400, 400]),
    dofs_idx_local = dofs_idx,
)
# set force range for safety
m0609.set_dofs_force_range(
    lower          = np.array([-87, -87, -87, -87, -12, -12]),
    upper          = np.array([ 87,  87,  87,  87,  12,  12]),
    dofs_idx_local = dofs_idx,
)


######## Main
# 90°	- 1.57
# 180°	- 3.14
# 270°	- 4.71
# 360°	- 6.28

m0609.set_dofs_position(np.array([0, 0, 0, 0, 0, 0]), dofs_idx)
m0609.set_dofs_position(np.array([0, 0]), gripper_idx)

for i in range(750):
    if i == 0:
        m0609.control_dofs_position(
            np.array([0, 0, 0, 0, 0, 0]),
            dofs_idx,
        )
        m0609.control_dofs_position(
            np.array([0,0]),
            gripper_idx,
        )
    elif i == 250:
        m0609.control_dofs_position(
            np.array([0, 0, 1.57, 0, 1.57, 0]),
            dofs_idx,
        )
        m0609.control_dofs_position(
            np.array([0.7,0.7]),
            gripper_idx,
        )
    elif i == 500:
        m0609.control_dofs_position(
            np.array([0, 0, 0, 0, 0, 0]),
            dofs_idx,
        )
        m0609.control_dofs_position(
            np.array([0.0,0.0]),
            gripper_idx,
        )
    scene.step()

while True:
    scene.step()
    m0609.control_dofs_position(
        np.array([0.0, 0, 0, 0, 0, 0]),
        dofs_idx,
    )
    m0609.control_dofs_position(
            np.array([0.0,0.0]),
            gripper_idx,
        )