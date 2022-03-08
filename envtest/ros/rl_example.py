#!/usr/bin/python3

import numpy as np
from utils import AgileCommand


def rl_example(state, obstacles):
    # Convert obstacles to vector observation
    obs_vec = []
    for obstacle in obstacles:
        obs_vec.append(obstacle.position.x)
        obs_vec.append(obstacle.position.y)
        obs_vec.append(obstacle.position.z)
        obs_vec.append(obstacle.scale)
    obs_vec = np.array(obs_vec)

    # Convert state to vector observation
    state_vec = []
    state_vec.append(state.pos[0])
    state_vec.append(state.pos[1])
    state_vec.append(state.pos[2])
    state_vec.append(state.vel[0])
    state_vec.append(state.vel[1])
    state_vec.append(state.vel[2])
    state_vec = np.array(state_vec)

    # TODO: Perform policy forward pass
    # TODO: @Yunlong add some example code here

    command_mode = 1
    command = AgileCommand(command_mode)
    command.t = state.t
    command.collective_thrust = 15.0
    command.bodyrates = [0.0, 0.0, 0.0]

    return command
