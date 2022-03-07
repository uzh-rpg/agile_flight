# Agile Flight

This repository holds the code for the ICRA22 competition on autonomous agile obstacle avoidance developed by the Robotics and Perception Group.
Use this code to design your own algorithms for high-speed obstacle avoidance and compare it with others!

The codebase provides the following functionalities:
1. Training utilities to use reinforcement learning for the task of high-speed obstacle avoidance. 
2. Simple testing scripts to evaluate your code in the Robot Operating System (ROS). If you don't want to use machine learning, use this script to easily iterate during algorithm development. 

All evaluation during the competition will be performed using the same ROS evaluation, but on previously unseen environments / obstacle configurations!



## Installation with ROS
You need to install this to run the evaluation. If you only want to train algorithms using reinforcement learning without evaluation, you can skip this step.
We only support Ubuntu 20.04 with ROS noetic. Other setups are likely to work as well but not actively supported.

Create a new catkin workspace. 
```
cd     # or wherever you'd like to install this code
export ROS_VERSION=noetic
export CATKIN_WS=./icra22_competition_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color

cd src
git clone git@github.com:uzh-rpg/agile_flight.git
cd agile_flight
```

Run the `setup_ros.bash` in the main folder of this repository, it will ask for sudo permissions.

```bash
./setup_ros.bash

catkin build
```

## Installation wtih Python 
If you want to train a learning-based obstacle avoidance policy using Python, you can use our training utilities for this! 
Run the `setup_py.bash` in the main folder of this repository, it will ask for sudo permissions.

(If you don't plan to use RL, python setup is not needed.)

```bash
./setup_py.bash
```

## Testing in ROS
Make sure you have completed the ROS Installation (todo: add link?) before!

Start the simulation:
```
roslaunch envsim visionenv_sim.launch render:=True
```

Start the navigation code:

I (Elia) started something in `envtest/ros/ros_test.py`. Not clear yet how to interface with trained policy from Yunlong?
Also not yet clear how we get the information about the obstacles in ROS, which is needed for all non-vision-based approaches.
```
TODO
```

## Usage with Python 

To test if the Flightmare simulator has been correctly installed as a python package, run vision demo script via the following command. 

```
cd agile_flight/envtest
python3 -m python.run_vision_demo --render 1
```

Read [this](/envtest/python/README.md) to know more about how to use the code for Python and Reinforcement Learning. 
