import torch
import math
import numpy as np
import genesis as gs
from genesis.utils.geom import quat_to_xyz, transform_by_quat, inv_quat, transform_quat_by_quat

from quadruped_controller import QuadrupedController

def gs_rand_float(lower, upper, shape, device):
    return (upper - lower) * torch.rand(size=shape, device=device) + lower

class Go2Env:
    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False, device="cuda"):
        self.device = torch.device(device)

        self.num_envs = num_envs #병렬로 실행할 환경 개수
        self.num_obs = obs_cfg["num_obs"] #로봇에게 input할 값의 개수, (로봇이 관찰하는)관찰값 개수
        self.num_privileged_obs = None #privileged 관찰값 개수
        self.num_actions = env_cfg["num_actions"] #로봇이 수행할 수 있는 행동 개수
        self.num_commands = command_cfg["num_commands"] #로봇이 받을 수 있는 명령 개수

        self.simulate_action_latency = True  # there is a 1 step latency on real robot
        self.dt = 0.02  # control frequency on real robot is 50hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt) #하나의 에피소드 길이 

        self.env_cfg = env_cfg #환경 관련 설정(액션개수, 종료조건, 초기 설정 등)
        self.obs_cfg = obs_cfg #관찰 관련 설정(로봇의 선속도, 각속도, 상대위치 등)
        self.reward_cfg = reward_cfg #보상관련 설정
        self.command_cfg = command_cfg #로봇이 받을 명령(목표)

        self.obs_scales = obs_cfg["obs_scales"] #관찰값 정규화
        self.reward_scales = reward_cfg["reward_scales"] #보상 정규화

        # create scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            #시간 간격과 substep으로 시뮬레이션 생성
            # substep이란 시간 적분을 위한 서브 스텝 수(더 정확한 물리 연산 수행) -> 시간 적분은 물체의 위치, 속도, 가속도를 시간에 따라 업데이트하는 과정
            # substep 참고
            # 드론 시뮬레이션(강체 동역학): substeps=2~4
            # 로봇 팔(정밀 조작): substeps=4~8
            # 유체 시뮬레이션: substeps=8~16
            viewer_options=gs.options.ViewerOptions(
                max_FPS=int(0.5 / self.dt),
                camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
            vis_options=gs.options.VisOptions(n_rendered_envs=1), #동시에 랜더링 하는 개수
            rigid_options=gs.options.RigidOptions(  #뉴턴 기반 강체 solver사용 / 충돌 감지 활성화 / 관절제한 활성화
                dt=self.dt,
                constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True,
                enable_joint_limit=True,
            ),
            show_viewer=show_viewer,
        )

        # add plain
        self.scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

        # add robot
        self.base_init_pos = torch.tensor(self.env_cfg["base_init_pos"], device=self.device)
        self.base_init_quat = torch.tensor(self.env_cfg["base_init_quat"], device=self.device)
        self.inv_base_init_quat = inv_quat(self.base_init_quat)
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file="urdf/go2/urdf/go2.urdf",
                pos=self.base_init_pos.cpu().numpy(),
                quat=self.base_init_quat.cpu().numpy(),
            ),
        )

        # build
        self.scene.build(n_envs=num_envs)

        # names to indices
        self.motor_dofs = [self.robot.get_joint(name).dof_idx_local for name in self.env_cfg["dof_names"]]

        num_dofs = len(self.motor_dofs)
        # PD control parameters
        force_lower_bound = np.array([-100] * len(self.motor_dofs))  # 최소 출력
        force_upper_bound = np.array([500] * len(self.motor_dofs))   # 최대 출력
        self.robot.set_dofs_force_range(
            lower=force_lower_bound,
            upper=force_upper_bound,
            dofs_idx_local=self.motor_dofs
        )
        self.robot.set_dofs_kp([self.env_cfg["kp"]] * num_dofs, self.motor_dofs)
        self.robot.set_dofs_kv([self.env_cfg["kd"]] * num_dofs, self.motor_dofs)

        self.controllers = [QuadrupedController() for i in range(self.num_envs)]

        # prepare reward functions and multiply reward scales by dt
        self.reward_functions, self.episode_sums = dict(), dict() # = 보상 함수를 저장하는 딕셔너리, 에피소드의 보상의 누적 합을 저장하는 딕셔너리.
        for name in self.reward_scales.keys(): # 요소들의 가중치를 가져옴
            self.reward_scales[name] *= self.dt # 시간 간격을 고려하여 가중치 조정 -> dt=0.01이면 보상이 100Hz(1초에 100번) 적용되므로 보상을 조정해야 함.
            self.reward_functions[name] = getattr(self, "_reward_" + name) #보상함수를 딕셔너리에 저장
            self.episode_sums[name] = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_float) #새로운 에피소드가 시작할 때 보상의 누적 합을 초기화

        # initialize buffers
        self.base_lin_vel = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float) #몸체의 선속도, 병렬 환경(num_envs)의 개수만큼 3D 속도 벡터(x,y,z) 생성
        self.base_ang_vel = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float) #각 환경마다 3축(x, y, z)을 기준으로 회전 속도를 저장.
        self.projected_gravity = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float) #로봇 기준 좌표계에서의 중력 벡터, (x, y, z)
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], device=self.device, dtype=gs.tc_float).repeat( #전역(월드 좌표계)에서의 중력 방향
            self.num_envs, 1
        )
        #z 방향(-1.0)으로 중력이 작용하는 일반적인 중력 가속도(9.81m/s²을 정규화하여 -1.0으로 표현).
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), device=self.device, dtype=gs.tc_float)  #로봇의 관찰값을 저장
        self.rew_buf = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_float) # 각 환경에서 받은 보상을 저장
        self.reset_buf = torch.ones((self.num_envs,), device=self.device, dtype=gs.tc_int) # 환경 리셋 여부(1이면 리셋, 0이면 계속)
        self.episode_length_buf = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_int) # 환경마다 경과한 시간
        self.commands = torch.zeros((self.num_envs, self.num_commands), device=self.device, dtype=gs.tc_float) # 로봇이 수행해야할 명령(위치, 속도, 자세 등)
        self.commands_scale = torch.tensor( # 명령값을 정규화하기 하기 위한 스케일을 저장
            [self.obs_scales["lin_vel"], ],
            device=self.device,
            dtype=gs.tc_float,
        )
        self.actions = torch.zeros((self.num_envs, self.num_actions), device=self.device, dtype=gs.tc_float) # 현재 로봇이 수행한 액션
        self.last_actions = torch.zeros_like(self.actions)  # 이전 시간 스텝에서 로봇이 수행한 액션
        # 현재&이전 액션 -> 로봇이 얼마나 부드럽게 움직이는지를 평가하는 스무딩 보상(Smooth Reward) 계산에 사용됨
        self.dof_pos = torch.zeros((self.num_envs, num_dofs), device=self.device, dtype=gs.tc_float) #	현재 관절 위치 (joint position)
        self.dof_vel = torch.zeros((self.num_envs, num_dofs), device=self.device, dtype=gs.tc_float) #	현재 관절 속도 (joint velocity)
        self.last_dof_vel = torch.zeros((self.num_envs, num_dofs), device=self.device, dtype=gs.tc_float) # 이전 스텝의 관절 속도
        self.base_pos = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float) # 로봇 베이스의 위치 (x, y, z)
        self.base_quat = torch.zeros((self.num_envs, 4), device=self.device, dtype=gs.tc_float) # 로봇 베이스의 자세 (쿼터니언)
        self.default_dof_pos = torch.tensor( # 초기 관절 위치 (로봇의 기본 포즈)
            [self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["dof_names"]],
            device=self.device,
            dtype=gs.tc_float,
        )
        self.robot.set_dofs_position(
            position=self.dof_pos * num_envs,
            dofs_idx_local=self.motor_dofs,
            zero_velocity=True,
        )

        self.x_lin_vel = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_float)

        self.last_base_pos = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)

        self.extras = dict()  # extra information for logging

    def _resample_commands(self, envs_idx):  # 로봇이 수행해야 하는 명령(command)를 생성함 (로봇의 목표)
        self.commands[envs_idx, 0] = gs_rand_float(*self.command_cfg["lin_vel_x_range"], (len(envs_idx),), self.device)
        self.commands[envs_idx, 1] = gs_rand_float(*self.command_cfg["lin_vel_y_range"], (len(envs_idx),), self.device)
        self.commands[envs_idx, 2] = gs_rand_float(*self.command_cfg["ang_vel_yaw_range"], (len(envs_idx),), self.device)

    def step(self, actions):
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"]) #행동의 범위 제한
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions #실행할 행동 결정
        # self.simulate_action_latency, 행동 지연(latency)을 시뮬레이션할지
        # 로봇의 실제 환경에서는 신호 전달 지연이 발생할 수 있음.
        # 로봇의 모터에 명령을 보내면 즉시 적용되지 않고 약간의 지연(delay)이 존재함.
        # 이를 반영하기 위해 simulate_action_latency=True인 경우 이전 스텝의 행동(last_actions)을 그대로 실행.
        # target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos # 목표 관절 위치 설정
        # 네트워크가 예측한 행동(exec_actions)에 스케일 값을 곱하여 실제 관절 움직임의 크기를 조정.
        # 기본적인 관절 위치(self.default_dof_pos)를 기준으로 행동값을 적용.
        # self.robot.control_dofs_position(target_dof_pos, self.motor_dofs) # 로봇을 실제로 움직임
        speed_torch = exec_actions[:, 0]
        lin_vel_x_torch = exec_actions[:, 1]
        lin_vel_y_torch = exec_actions[:, 2]
        ang_vel_z_torch = exec_actions[:, 3]
        # swing_height_torch = exec_actions[:, 4]
        # stance_depth_torch = exec_actions[:, 5]
        # stance_duration_torch = exec_actions[:, 6]
        # nominal_height_torch = exec_actions[:, 7]
        
        positions_list = []

        for i in range(self.num_envs):
            speed = speed_torch[i]
            lin_vel_x = lin_vel_x_torch[i]
            lin_vel_y = lin_vel_y_torch[i]
            ang_vel_z = ang_vel_z_torch[i]
            # swing_height = swing_height_torch[i]
            # stance_depth = stance_depth_torch[i]
            # stance_duration = stance_duration_torch[i]
            # nominal_height = nominal_height_torch[i]
            
            # 각 env의 QuadrupedController에 보행 파라미터를 적용
            self.controllers[i].setVelocityCommand(lin_vel_x, lin_vel_y, ang_vel_z)
            self.controllers[i].setSpeedValue(speed)
            # self.controllers[i].setSwingHeight(swing_height)
            # self.controllers[i].setStanceDepth(stance_depth)
            # self.controllers[i].setStanceDuration(stance_duration)
            # self.controllers[i].setNominalHeight(nominal_height)
            # Controller가 계산한 관절 각도 가져오기
            pos_i = self.controllers[i].getJointPositions()
            positions_list.append(pos_i)


        # positions_array : shape (num_envs, n_joints)
        positions_array = np.array(positions_list, dtype=np.float32)

        # 5) 병렬 모드로 모든 env의 관절을 한꺼번에 제어
        #    (envs_idx=None 이면 전체 env에 대해 shape (num_envs, n_joints)를 적용)
        self.robot.control_dofs_position(
            position=positions_array,
            dofs_idx_local=self.motor_dofs,
            envs_idx=None
        )

        self.scene.step()

        # update buffers
        self.episode_length_buf += 1 # 에피소드 스텝 저장 -> threshold값을 넘으면 자동 종료
        self.base_pos[:] = self.robot.get_pos() # pos update
        self.last_base_pos[:] = self.base_pos[:]
        self.base_quat[:] = self.robot.get_quat() # pos update
        self.base_euler = quat_to_xyz( #오일려 변환
            transform_quat_by_quat(torch.ones_like(self.base_quat) * self.inv_base_init_quat, self.base_quat)
        )
        inv_base_quat = inv_quat(self.base_quat) # 역 쿼터니안 업데이트, 월드 좌표계를 로봇의 로컬 좌표계로 변환하는 데 사용.
        self.base_lin_vel[:] = transform_by_quat(self.robot.get_vel(), inv_base_quat) #월드 좌표계의 선속도를 로봇 좌표계로 변환하여 업데이트
        self.base_ang_vel[:] = transform_by_quat(self.robot.get_ang(), inv_base_quat) #월드 좌표계의 각속도를 로봇 좌표계로 변환하여 업데이트
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_base_quat) #월드 좌표계의 중력을 로봇 좌표계로 변환하여 업데이트
        self.dof_pos[:] = self.robot.get_dofs_position(self.motor_dofs) #각 관절의 현재 위치 업데이트
        self.dof_vel[:] = self.robot.get_dofs_velocity(self.motor_dofs) #각 관절의 현재 속도 업데이트

        # resample commands
        envs_idx = (
            (self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)
            .nonzero(as_tuple=False)
            .flatten()
        ) #로봇의 목표를 재생성해야 할 idx를 return
        self._resample_commands(envs_idx) # 만약 도달한 환경이 있다면 로봇이 수행해야할 목표 재생성, 환경 초기화는 아니고 다음 목표로 이동하도록 하는 것임

        # check termination and reset
        self.reset_buf = self.episode_length_buf > self.max_episode_length #환경을 리셋해야 하는 인덱스를 저장하는 텐서
        # 현재 환경의 스텝 수가 최대 허용 스텝(max_episode_length)을 초과하면 종료
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"] #로봇이 앞뒤로 기울어지면 종료
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"] #로봇이 좌우로 기울어지면 종료
        
        #시간초과된 환경은 로그 기록을 위해 저장(그냥 로그용)
        time_out_idx = (self.episode_length_buf > self.max_episode_length).nonzero(as_tuple=False).flatten()
        self.extras["time_outs"] = torch.zeros_like(self.reset_buf, device=self.device, dtype=gs.tc_float)
        self.extras["time_outs"][time_out_idx] = 1.0
        
        # 종료된 환경들에 대해 리셋
        self.reset_idx(self.reset_buf.nonzero(as_tuple=False).flatten())

        # compute reward
        self.rew_buf[:] = 0.0 # 현재 스텝의 보상을 저장하는 버퍼
        for name, reward_func in self.reward_functions.items(): #모든 보상함수를 실행하여 현재 스텝의 보상 버퍼와 에피소드의 보상 누적 합에 추가한다.
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # compute observations
        self.obs_buf = torch.cat( # 관찰값을 하나의 텐서로 변환 / 로봇에게 입력되는 값
            [
                #관찰값, 모델인풋
                self.actions * self.obs_scales["actions"], #1
                self.base_lin_vel[:, 0:1] * self.obs_scales["lin_vel"]#1
            ],
            axis=-1,
        )

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]

        return self.obs_buf, None, self.rew_buf, self.reset_buf, self.extras

    def get_observations(self):
        return self.obs_buf

    def get_privileged_observations(self):
        return None

    # 각종 환경에 대한 변수를 초기화
    def reset_idx(self, envs_idx):
        if len(envs_idx) == 0:
            return
        for i in envs_idx:
            self.controllers[i].setSpeedValue(0.0)
            self.controllers[i].setVelocityCommand(0, 0, 0)

        # reset dofs
        self.dof_pos[envs_idx] = self.default_dof_pos
        self.dof_vel[envs_idx] = 0.0
        self.robot.set_dofs_position(
            position=self.dof_pos[envs_idx],
            dofs_idx_local=self.motor_dofs,
            zero_velocity=True,
            envs_idx=envs_idx,
        )

        # reset base
        self.base_pos[envs_idx] = self.base_init_pos
        self.last_base_pos[:] = self.base_pos[:]
        self.base_quat[envs_idx] = self.base_init_quat.reshape(1, -1)
        self.robot.set_pos(self.base_pos[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.robot.set_quat(self.base_quat[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.base_lin_vel[envs_idx] = 0
        self.base_ang_vel[envs_idx] = 0
        self.robot.zero_all_dofs_velocity(envs_idx)

        # reset buffers
        self.last_actions[envs_idx] = 0.0
        self.last_dof_vel[envs_idx] = 0.0
        self.episode_length_buf[envs_idx] = 0
        self.reset_buf[envs_idx] = True

        # fill extras
        self.extras["episode"] = {}
        for key in self.episode_sums.keys():
            self.extras["episode"]["rew_" + key] = (
                torch.mean(self.episode_sums[key][envs_idx]).item() / self.env_cfg["episode_length_s"]
            )
            self.episode_sums[key][envs_idx] = 0.0

        self._resample_commands(envs_idx)

    # 전체 환경 초기화(진짜 실행하기 전에 한번 하는 것 같음)
    def reset(self):
        self.reset_buf[:] = True
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        return self.obs_buf, None

    # ------------ reward functions----------------
    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        # 로봇이 요구된 선속도(commands[:, :2])를 정확하게 따라가도록 보상을 부여.
        # 오차가 작을수록 보상을 크게 하기 위해 지수 함수(exp)를 사용하여 부드럽게 감소.
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_no_movement_penalty(self):
        # 현재 스텝과 이전 스텝의 x축 위치 차이 계산
        x_pos_diff = torch.abs(self.base_pos[:, 0] - self.last_base_pos[:, 0])
        # 위치 변화가 거의 없으면 패널티 부여 (0.01m 이하 움직이면 패널티)
        penalty = torch.where(x_pos_diff < self.reward_cfg["no_movement_threshold"], torch.tensor(1.0, device=self.device), torch.tensor(0.0, device=self.device))
        return penalty

    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_ang_vel_yaw(self):
        # Penalize yaw angular velocity (회전을 억제)
        return torch.square(self.base_ang_vel[:, 2])