#!/usr/bin/python3

import torch
import numpy as np

# 
from ruamel.yaml import YAML
from utils import AgileCommand
from scipy.spatial.transform import Rotation as R

# stable baselines 
from stable_baselines3.common.utils import get_device
from stable_baselines3.ppo.policies import MlpPolicy

class RPGMlpPolicy(torch.nn.Module):

    def __init__(self, save_variables, device):
        super(RPGMlpPolicy, self).__init__()
        self.policy = MlpPolicy(**save_variables)
        self.policy.action_net = torch.nn.Sequential(self.policy.action_net,
                                                     torch.nn.Tanh())
        self.device = device
    
    def __call__(self, obs: torch.Tensor) -> torch.Tensor:
        return self.forward(obs)

    def load_weights(self, state_dict):
        self.policy.load_state_dict(state_dict, strict=False)
        self.policy.to(device=self.device)

    def forward(self, obs: torch.Tensor):
        return self.policy._predict(obs, deterministic=True)
        

def normalize_obs(obs, obs_mean, obs_var):
    return (obs - obs_mean) / np.sqrt(obs_var + 1e-8)

def rl_example(state, obstacles):
    policy, obs_mean, obs_var, act_mean, act_std, device = load_rl_policy()
    # Convert obstacles to vector observation
    obs_vec = []
    for obstacle in obstacles:
        obs_vec.append(obstacle.position.x)
        obs_vec.append(obstacle.position.y)
        obs_vec.append(obstacle.position.z)
        obs_vec.append(obstacle.scale)
    obs_vec = np.array(obs_vec)

    # Convert state to vector observation
    goal_pos = np.array([80.0, 0.0, 0.0]) 
    delta_goal = state.pos - goal_pos
    att_aray = np.array([state.att[1], state.att[2], state.att[3], state.att[0]])
    rotation_matrix = R.from_quat(att_aray).as_matrix().reshape((9,), order="F")

    # constructe observation and perform normalization
    obs = np.concatenate([
        delta_goal, rotation_matrix, state.vel, state.omega, obs_vec], axis=0).astype(np.float32) 
    norm_obs = normalize_obs(obs, obs_mean, obs_var)

    #  compute action
    obs = torch.as_tensor(obs).to(device)
    action = policy(norm_obs).detach().cpu().numpy()
    action = (action * act_std + act_mean)[0, :]

    command_mode = 1
    command = AgileCommand(command_mode)
    command.t = state.t
    command.collective_thrust = action[0] 
    command.bodyrates = action[1:4] 

    return command

def load_rl_policy():
    ppo_dir = "/home/yunlong/Projects/ws_agile/src/agile_flight/envtest/python/saved/PPO_3"
    policy_dir = ppo_dir + "/Policy/iter_00100.pth" 
    rms_dir = ppo_dir + "/RMS/iter_00100.npz" 
    cfg_dir =  ppo_dir + "/config.yaml"

    # action 
    env_cfg = YAML().load(open(cfg_dir, "r"))
    quad_mass = env_cfg["quadrotor_dynamics"]["mass"]
    omega_max = env_cfg["quadrotor_dynamics"]["omega_max"]
    thrust_max = 4 * env_cfg["quadrotor_dynamics"]["thrust_map"][0] * \
        env_cfg["quadrotor_dynamics"]["motor_omega_max"] * \
        env_cfg["quadrotor_dynamics"]["motor_omega_max"]
    act_mean = np.array([thrust_max / quad_mass / 2, 0.0, 0.0, 0.0])[np.newaxis, :] 
    act_std = np.array([thrust_max / quad_mass / 2, \
       omega_max[0], omega_max[1], omega_max[1]])[np.newaxis, :] 


    rms_data = np.load(rms_dir)
    obs_mean = np.mean(rms_data["mean"], axis=0)
    obs_var = np.mean(rms_data["var"], axis=0 )

    device = get_device("cpu")

    # -- load saved varaiables 
    saved_variables = torch.load(policy_dir, map_location=device)
    obs_dim  = saved_variables["data"]["observation_space"].shape[0]

    # create policy
    policy = RPGMlpPolicy(saved_variables["data"], device)
    policy.load_weights(saved_variables["state_dict"])

    # policy just in time compliation
    dummy_inputs = torch.rand(1, obs_dim, device=device)
    policy = torch.jit.trace(policy, dummy_inputs)
    return policy, obs_mean, obs_var, act_mean, act_std, device