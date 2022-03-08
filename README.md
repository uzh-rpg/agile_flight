# DodgeDrone: Vision-based Agile Drone Flight (ICRA 2022 Competition)

![Intro](https://uzh-rpg.github.io/icra2022-dodgedrone/assets/intro_image.png)


Would you like to push the boundaries of drone navigation? Then participate in the dodgedrone competition!
You will get the chance to develop perception and control algorithms to navigate a drone in both static and dynamic environments. Competing in the challenge will deepen your expertise in computer vision and control, and boost your research.
You can find more information at the [competition website](https://uzh-rpg.github.io/icra2022-dodgedrone/).

This codebase provides the following functionalities:
1. A simple high-level API to evaluate your navigation policy in the Robot Operating System (ROS). This is completely independent on how you develop your algorithm. 
2. Training utilities to use reinforcement learning for the task of high-speed obstacle avoidance. 

All evaluation during the competition will be performed using the same ROS evaluation, but on previously unseen environments / obstacle configurations.


## Flight API

This library contains the core of our testing API. It will be used for evaluating all submitted policies. The API is completely independent on how you build your navigation system. You could either use our reinforcement learning interface (more on this below) or add your favourite navigation system.

### Prerequisite
Before continuing, make sure to have g++ and gcc to version 9.3.0. You can check this by typing in a terminal `gcc --version` and `g++ --version`. Follow [this guide](https://linuxize.com/post/how-to-install-gcc-compiler-on-ubuntu-18-04/) if your compiler is not compatible.

In addition, make sure to have ROS installed. Follow [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu) and install ROS Noetic if you don't already have it.

### Installation
We only support Ubuntu 20.04 with ROS noetic. Other setups are likely to work as well but not actively supported.

Start by creating a new catkin workspace. 
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

Run the `setup_ros.bash` in the main folder of this repository, it will ask for sudo permissions. Then build the packages.

```bash
./setup_ros.bash

catkin build
```

### Usage: TODO

TODO: How can I use this? Is there some form of documentation?

## Training (Optional)
We also provide an easy interface for training your navigation policy using RL. While this is not required to compete, it could just make your job easier if you plan on using RL.

### Installation
Run the `setup_py.bash` in the main folder of this repository, it will ask for sudo permissions.

```bash
./setup_py.bash
```

### Usage

Follow [this guide](/envtest/python/README.md) to know more about how to use the training code.

### Testing: TODO

(Antonio) How should I use this in the previously mentioned ROS installation? Added this below, but not sure how correct this is.


Make sure you have completed the installation of the flight API before continuing.

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


## TODOs

[] Double check the command code, which has caused some issues
[] ROS launch file for RL policy

