## Installation Test

To test if the Flightmare simulator has been correctly installed as a python package, run vision demo script via the following command. 

```
cd agile_flight/envtest
python3 -m python.run_vision_demo --render 1
```

You should have TWO opencv windows open: one RGB image and one depth image. 

(Antonio) Why is this relevant?


Each [vision_env](/flightmare/flightlib/include/flightlib/envs/vision_env/vision_env.hpp) simulate one quadrotor, which has a monocular camera attached. We use vectorized environment [vision_vec_env](/flightmare/flightlib/include/flightlib/envs/vision_env/vision_env.hpp) for parallel simulation. As a result, you will have multiple simulated cameras. 

![vision_demo](/docs/imgs/vision_demo.png)

## Policy Training  

We provide a simple reinforcement learning code for you. Run the training via the following command. 

```
cd agile_flight/envtest
python3 -m python.run_vision_ppo --render 0 --train 1
```
## Policy Evaluation  

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

 
## An overview of Reinforcement Learning for Obstacle Avoidance 

![vision_demo](/docs/imgs/env_ppo.png)

### About the RL environment
The RL environment is specified in 

```
flightmare/flightlib/include/flightlib/env/vision_env
```

which defines a training environment for reinforcement learning.
It simulates the quadrotor dynamics and the obstacles.  

Our code provide only a basic implementation for the task. The performance of current RL policy is sub-optimal.
It is highly recommanded that you make significant changes to the environment in order to train a policy effectively for obstacle avoidance.
For example, design a better reward function and initialization strategy.

### About the RL Algorithm 
We use [stable-baselines3](https://github.com/DLR-RM/stable-baselines3) for the reinforcement learning. Specifically, our code provide interface to the PPO algorithm, since it allows parallelizing several hundreds of environment for training. 
## Policy Evaluation in ROS

Follow the steps on [this guide](https://github.com/uzh-rpg/agile_flight/blob/main/README.md#testing-todo) to evaluate your policies with our flight stack API.
