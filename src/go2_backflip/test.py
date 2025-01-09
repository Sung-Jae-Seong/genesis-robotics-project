import os
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import genesis as gs

# 환경 변수 설정
os.environ['PYOPENGL_PLATFORM'] = 'glx'

class Ros2ToGenesisController(Node):
    def __init__(self):
        super().__init__('ros2_to_genesis_controller')

        # ROS 2 Subscriber 설정
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            self.joint_trajectory_callback,
            10
        )

        # Genesis 초기화 및 Scene 설정
        gs.init(backend=gs.cpu)
        self.scene = gs.Scene(
            show_viewer=True,
            sim_options=gs.options.SimOptions(
                dt=0.01,  # 시뮬레이션 타임스텝
            ),
        )

        self.scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

        #robot_path = "/home/sjs/genesis-robotics-project/resource/go2/urdf/go2_description.urdf"
        robot_path = "urdf/go2/urdf/go2.urdf"
        # URDF 로봇 추가
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=robot_path,
                pos=(0.0, 0.0, 0.6),
            )
        )
        self.robot.set_friction(0.1)
        
        self.scene.build()

        # 관절 정의
        self.jnt_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
        ]
        self.jnt_idx = [self.robot.get_joint(name).dof_idx_local for name in self.jnt_names]

        # 모터 출력 강화: 힘 범위 설정
        force_lower_bound = np.array([-100] * len(self.jnt_idx))  # 최소 출력
        force_upper_bound = np.array([500] * len(self.jnt_idx))   # 최대 출력
        self.robot.set_dofs_force_range(
            lower=force_lower_bound,
            upper=force_upper_bound,
            dofs_idx_local=self.jnt_idx
        )

        # 모터 출력 강화: 위치 게인 (Kp) 설정
        position_gains = np.array([5000 * 0.05] * len(self.jnt_idx))
        self.robot.set_dofs_kp(
            kp=position_gains,
            dofs_idx_local=self.jnt_idx
        )

        # 모터 출력 강화: 속도 게인 (Kv) 설정
        velocity_gains = np.array([600 * 0.01] * len(self.jnt_idx))
        self.robot.set_dofs_kv(
            kv=velocity_gains,
            dofs_idx_local=self.jnt_idx
        )

        self.get_logger().info('Ros2ToGenesisController initialized.')

    def joint_trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warning('Received JointTrajectory message with no points.')
            return

        # 첫 번째 point의 position 값 추출
        positions = msg.points[0].positions
        if len(positions) != len(self.jnt_idx):
            self.get_logger().error('Mismatch in joint names and positions.')
            return

        # Genesis 로봇에 목표 위치 설정
        self.robot.control_dofs_position(
            position=np.array(positions),
            dofs_idx_local=self.jnt_idx
        )

        # 시뮬레이션 한 스텝 진행
        self.scene.step()

def main(args=None):
    rclpy.init(args=args)
    node = Ros2ToGenesisController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
