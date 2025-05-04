import argparse
import os
import pickle
import shutil

from go2_env import Go2Env
from rsl_rl.runners import OnPolicyRunner

import genesis as gs


def get_train_cfg(exp_name, max_iterations):

    train_cfg_dict = {
        "algorithm": {
            "clip_param": 0.15, # PPO의 클리핑 파라미터 (0.2 → 급격한 학습 제한)
            "desired_kl": 0.01, # KL divergence = 새로운 정책과 기존 정책 간의 차이 / ex. 0.01보다 크다면 학습률을 낮춤(학습 안정화) 
            "entropy_coef": 0.004, # 정책 탐색 다양성을 위한 엔트로피 보상 계수 (다양한 행동 시도)
            "gamma": 0.99, # 0.99 → 미래의 보상 고려
            "lam": 0.95,  # 1.0 : fully montecarlo -> 먼 미래 반영 / 0.0 : fullt TD -> 단기적인 보상 (주로 0.9~0.97 사이)
            # montecarlo : 에피소드가 끝나면 모든 보상을 누적하여 한 번에 학습(편향 방지)
            # TD : 다음 상태에서 보상을 추정하여 가중치 업데이트
            "learning_rate": 0.0005,
            "max_grad_norm": 1.0,  # 그래디언트 클리핑 값 (1.0) -> 급격한 학습 방지
            "num_learning_epochs": 5, # 학습 반복 횟수 -> 데이터 수집 후 학습 횟수
            "num_mini_batches": 4, # 미니배치 개수
            "schedule": "adaptive", # 학습률 스케줄링 방식 -> adaptive : KL divergence를 기반으로 학습률을 조절
            "use_clipped_value_loss": True, # 값 함수 손실을 클리핑할지 여부, 급격한 학습 제한
            "value_loss_coef": 1.0, # 값 함수 손실 계수 1.0 : 값 함수 손실을 정책 손실과 동일한 중요도로 고려.
            # value function : 각 상태에서의 기대 보상
            # V(s) - 상태 가치 함수 - 현재 상태에서의 앞으로의 기대 보상
            # Q(s, a) - 행동 가치 함수 - 현재 상태에서 어떤 행동을 했을 때 기대 보상
        },
        "init_member_classes": {}, # 사용자가 임의로 추가할 강화학습 모듈이나 알고리즘...
        "policy": {
            "activation": "elu", # 활성 함수
            "actor_hidden_dims": [256, 128], # 정책 네트워크 크기 -> 정책(행동)을 선택
            "critic_hidden_dims": [256, 128], # 가치 네트워크 크기 -> 현재 상태에 대한 판단
            "init_noise_std": 1.0, # 초기 노이즈 표준편차 (탐색을 위해 초기 액션 분포에 적용, 보통 init_noise_std < 2.0)
        },
        "runner": {
            "algorithm_class_name": "PPO",
            "checkpoint": -1, # 체크포인트 사용안하고 초기 상태에서 시작
            "experiment_name": exp_name,
            "load_run": -1, # 기존 학습을 불러오지 않고 새로운 학습 진행
            "log_interval": 1, # 1회 학습마다 로그 출력
            "max_iterations": max_iterations,
            "num_steps_per_env": 24,  # 환경당 100스텝 학습 후 업데이트 진행
            "policy_class_name": "ActorCritic", # 사용되는 정책 네트워크는 Actor-Critic 구조
            "record_interval": -1, # 시뮬레이션 녹화 비활성화
            "resume": False, # 이전 학습을 이어서 하지 않음
            "resume_path": None, # 체크포인트 경로 없음
            "run_name": "",
            "runner_class_name": "runner_class_name",
            "save_interval": 100,
        },
        "runner_class_name": "OnPolicyRunner",
        "seed": 1, # 랜덤 시드 고정 (실험 재현성 보장)
    }

    return train_cfg_dict


def get_cfgs():
    env_cfg = {
        "num_actions": 8, # 12개의 joint -> quadruped_controller로 대체
        # joint/link names
        "default_joint_angles": {  # [rad]
            # 주어진 배열 12개 값이 dof_names 순서에 정확히 대응되도록 매핑
            "FR_hip_joint":   1.5893254712295857e-08,
            "FR_thigh_joint": 1.0143535137176514,
            "FR_calf_joint": -2.0287070274353027,
            "FL_hip_joint":  -1.0331603306212855e-07,
            "FL_thigh_joint": 1.0143535137176514,
            "FL_calf_joint": -2.0287070274353027,
            "RR_hip_joint":   1.5893254712295857e-08,
            "RR_thigh_joint": 1.0143535137176514,
            "RR_calf_joint": -2.0287070274353027,
            "RL_hip_joint":  -1.0331603306212855e-07,
            "RL_thigh_joint": 1.0143535137176514,
            "RL_calf_joint": -2.0287070274353027,
        },
        "dof_names": [
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ],
        # PD
        "kp": 250.0,
        "kd": 6,
        # termination -> 경로를 크게 벗어나는 것 추가
        "termination_if_roll_greater_than": 45,  # degree
        "termination_if_pitch_greater_than": 90,
        # base pose
        "base_init_pos": [0.0, 0.0, 0.4],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25, #수정
        "simulate_action_latency": True,
        "clip_actions": 5.0, #수정
    }
    obs_cfg = {
        "num_obs": 23, #로봇이 관찰할 파라미터 개수
        "obs_scales": {  #예를 들어, 속도 값이 3.0인데 관절 위치 값이 0.1이면, 신경망이 상대적으로 속도를 더 중요한 값으로 인식할 가능성이 높아짐.
            "lin_vel" : 1.0,
            "ang_vel" : 1.0,
            "euler" : 1.0,
            "actions" : 1.0
        },
    }
    reward_cfg = { # 경로 도착이나 관련 보상 추가
        "tracking_sigma": 0.25,
        "no_movement_threshold": 0.01,
        "reward_scales": {
            "tracking_lin_vel" : 1.0,
            "tracking_ang_vel" : 1.0,
            "no_movement_penalty": -100,
            "lin_vel_z" : -5.0,
            "ang_vel_yaw": -0.1
        },
    }
    command_cfg = { # 로봇이 받을 명령 -> 경로
        "num_commands": 3,
        "lin_vel_x_range": [0.3, 0.3], #명령의 범위
        "lin_vel_y_range": [0.0, 0.0],
        "ang_vel_yaw_range": [0.0, 0.0],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-walking")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("--max_iterations", type=int, default=100)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs()
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    env = Go2Env(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()

"""
# training
python3 go2_train.py -B 1024 --max_iterations 2
"""