## Installation Test

To test if the Flightmare simulator has been correctly installed as a python package, run vision demo script via the following command. 

```
cd agile_flight/envtest
python3 -m python.run_vision_demo --render 1
```

You should have TWO opencv windows open: one RGB image and one depth image. 


![vision_demo](/docs/imgs/vision_demo.png)

## An overview of Reinforcement Learning for Obstacle Avoidance 

![vision_demo](/docs/imgs/env_ppo.png)

### About the RL environment
The RL environment is declared in [this file](https://github.com/uzh-rpg/flightmare/blob/ee30f203df42596668ee4c386ce1e45aedeedb8c/flightlib/include/flightlib/envs/vision_env/vision_env.hpp). 

Edit the file [vision_env](https://github.com/uzh-rpg/flightmare/blob/ee30f203df42596668ee4c386ce1e45aedeedb8c/flightlib/src/envs/vision_env/vision_env.cpp) to change the [reward function](https://github.com/uzh-rpg/flightmare/blob/ee30f203df42596668ee4c386ce1e45aedeedb8c/flightlib/src/envs/vision_env/vision_env.cpp#L288), the [terminal condition](https://github.com/uzh-rpg/flightmare/blob/ee30f203df42596668ee4c386ce1e45aedeedb8c/flightlib/src/envs/vision_env/vision_env.cpp#L330), or adding other environment functions. This file is used for simulating one quadrotor (physics, sensing, and obstacles) with a monocular camera attached.
The vectorized environment file [vision_vec_env](https://github.com/uzh-rpg/flightmare/blob/ee30f203df42596668ee4c386ce1e45aedeedb8c/flightlib/src/envs/vision_env/vision_vec_env.cpp) controls parallel simulation of multiple quadrotors. You might want to edit this file for more in depth changes to the simulation environment. 

Our code provide only a basic implementation for the task. The performance of current RL policy is sub-optimal.
It is highly recommanded that you make significant changes to the environment in order to train a policy effectively for obstacle avoidance.
For example, design a better reward function and initialization strategy.
You can take inspirations from our previous publication about how to [use PPO to solve a drone racing task](https://arxiv.org/abs/2103.08624)

### About the RL Algorithm 
We use [stable-baselines3](https://github.com/DLR-RM/stable-baselines3) for the reinforcement learning. 
Specifically, our code provide interface to the PPO algorithm, since it allows parallelizing several hundreds of environment for training. 

### Policy Training  

We provide a simple reinforcement learning code for you. Run the training via the following command. 

```
cd agile_flight/envtest
python3 -m python.run_vision_ppo --render 0 --train 1
```
### Policy Evaluation  

After training, you can test the trained policy by the following command.
```
python3 -m python.run_vision_ppo --render 0 --train 0 --trial trial_num --iter iter_num 
```
Depends on which checkpoint you want to load, change the **trail_num** and the **iter_num**.
All the neural network checkpoints are stored under /envtest/python/saved.
For example, 
```
python3 -m python.run_vision_ppo --render 0 --train 0 --trial 1 --iter 500 
```
for the policy that was trained for 500 iterations in PPO_1.



## Policy Evaluation in ROS

Follow the steps on [this guide](https://github.com/uzh-rpg/agile_flight/blob/main/README.md#testing-todo) to evaluate your policies with our flight stack API.
