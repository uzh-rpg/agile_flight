#!/usr/bin/python3

import os

import torch
import numpy as np

# 
from ruamel.yaml import YAML
from utils import AgileCommand
from scipy.spatial.transform import Rotation as R

# stable baselines 
from stable_baselines3.common.utils import get_device
from stable_baselines3.ppo.policies import MlpPolicy

def normalize_obs(obs, obs_mean, obs_var):
    return (obs - obs_mean) / np.sqrt(obs_var + 1e-8)

def rl_example(state, obstacles, rl_policy=None):
    policy, obs_mean, obs_var, act_mean, act_std = rl_policy
    # Convert obstacles to vector observation
    obs_vec = []
    for obstacle in obstacles.obstacles:
        obs_vec.append(obstacle.position.x)
        obs_vec.append(obstacle.position.y)
        obs_vec.append(obstacle.position.z)
        obs_vec.append(obstacle.scale)
    obs_vec = np.array(obs_vec)

    # Convert state to vector observation
    goal_vel = np.array([3.0, 0.0, 0.0]) 

    att_aray = np.array([state.att[1], state.att[2], state.att[3], state.att[0]])
    rotation_matrix = R.from_quat(att_aray).as_matrix().reshape((9,), order="F")
    obs = np.concatenate([
        goal_vel, rotation_matrix, state.vel, obs_vec], axis=0).astype(np.float64)

    obs = obs.reshape(-1, obs.shape[0])
    norm_obs = normalize_obs(obs, obs_mean, obs_var)
    #  compute action
    action, _ = policy.predict(norm_obs, deterministic=True)
    action = (action * act_std + act_mean)[0, :]

    command_mode = 1
    command = AgileCommand(command_mode)
    command.t = state.t
    command.collective_thrust = action[0] 
    command.bodyrates = action[1:4] 
    return command

def load_rl_policy(policy_path):
    policy_dir = policy_path  + "/Policy/iter_00500.pth" 
    rms_dir = policy_path + "/RMS/iter_00500.npz" 
    cfg_dir =  policy_path + "/config.yaml"

    # action 
    env_cfg = YAML().load(open(cfg_dir, "r"))
    quad_mass = env_cfg["quadrotor_dynamics"]["mass"]
    omega_max = env_cfg["quadrotor_dynamics"]["omega_max"]
    thrust_max = 4 * env_cfg["quadrotor_dynamics"]["thrust_map"][0] * \
        env_cfg["quadrotor_dynamics"]["motor_omega_max"] * \
        env_cfg["quadrotor_dynamics"]["motor_omega_max"]
    act_mean = np.array([thrust_max / quad_mass / 2, 0.0, 0.0, 0.0])[np.newaxis, :] 
    act_std = np.array([thrust_max / quad_mass / 2, \
       omega_max[0], omega_max[1], omega_max[2]])[np.newaxis, :] 

    rms_data = np.load(rms_dir)
    obs_mean = np.mean(rms_data["mean"], axis=0)
    obs_var = np.mean(rms_data["var"], axis=0)

    # # -- load saved varaiables 
    device = get_device("auto")
    saved_variables = torch.load(policy_dir, map_location=device)
    # Create policy object
    policy = MlpPolicy(**saved_variables["data"])
    #
    policy.action_net = torch.nn.Sequential(policy.action_net, torch.nn.Tanh())
    # Load weights
    policy.load_state_dict(saved_variables["state_dict"], strict=False)
    policy.to(device)

    return policy, obs_mean, obs_var, act_mean, act_std
