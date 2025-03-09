import os
import numpy as np
import genesis as gs

from quadruped_controller import QuadrupedController

controller = QuadrupedController()
controller.setURDFfromFile("robot.urdf")
controller.setGaitConfig("gait_config.yaml")
controller.setJointsMap("joints_map.yaml")
controller.setLinksMap("links_map.yaml")
joint_names = controller.getJointNames()

os.environ['PYOPENGL_PLATFORM'] = 'glx'

gs.init()
scene = gs.Scene(
    show_viewer=True,
    sim_options=gs.options.SimOptions(
        dt=0.01,  # 시뮬레이션 타임스텝
    ),
)

scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

#robot_path = "/home/sjs/genesis-robotics-project/resource/go2/urdf/go2_description.urdf"
robot_path = "urdf/go2/urdf/go2.urdf"
robot = scene.add_entity(
    gs.morphs.URDF(
        file=robot_path,
        pos=(0.0, 0.0, 0.6),
    )
)
robot.set_friction(0.1)

scene.build()

# 관절 정의
jnt_names = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]
jnt_idx = [robot.get_joint(name).dof_idx_local for name in jnt_names]

# 모터 출력 강화: 힘 범위 설정
force_lower_bound = np.array([-100] * len(jnt_idx))  # 최소 출력
force_upper_bound = np.array([500] * len(jnt_idx))   # 최대 출력
robot.set_dofs_force_range(
    lower=force_lower_bound,
    upper=force_upper_bound,
    dofs_idx_local=jnt_idx
)

# 모터 출력 강화: 위치 게인 (Kp) 설정
position_gains = np.array([5000 * 0.05] * len(jnt_idx))
robot.set_dofs_kp(
    kp=position_gains,
    dofs_idx_local=jnt_idx
)

# 모터 출력 강화: 속도 게인 (Kv) 설정
velocity_gains = np.array([600 * 0.01] * len(jnt_idx))
robot.set_dofs_kv(
    kv=velocity_gains,
    dofs_idx_local=jnt_idx
)

controller.setVelocityCommand(0, 0, 0)

import time
start = time.time()

last_time = time.time() 
while True:
# 첫 번째 point의 position 값 추출
    positions = controller.getJointPositions()

    # Genesis 로봇에 목표 위치 설정
    robot.control_dofs_position(
        position=np.array(positions),
        dofs_idx_local=jnt_idx
    )
    if time.time() - start > 5:
        controller.setVelocityCommand(1, 0, 0)
        controller.setSpeedValue(0.1)

    if time.time() - start > 10:
        controller.setSpeedValue(0.5)

    scene.step()
