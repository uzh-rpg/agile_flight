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

### Usage
The usage of this code base entails two main aspects: writing your algorithm and testing it in the simulator. 

**Writing your algorithm:**

To facilitate coding of your algorithms, we provided a simple code structure for you, just edit the following file: [envtest/ros/user_code.py](https://github.com/uzh-rpg/agile_flight/blob/main/envtest/ros/user_code.py). 
This file contains two functions, [compute_command_vision_based](https://github.com/uzh-rpg/agile_flight/blob/main/envtest/ros/user_code.py#L8) and [compute_command_state_based](https://github.com/uzh-rpg/agile_flight/blob/main/envtest/ros/user_code.py#L44).
In he vision-based case, you will get the current image and state of the quadrotor. In the state-based case, you will get the metric distance to obstacles and the state of the quadrotor. We strongly reccomend using the state-based version to start with, it is going to be much easier than working with pixels!

Depending on the part of the competition you are interested in, adapt the corresponding function.
To immediately see something moving, both functions at the moment publish a command to fly straight forward, of course without avoiding any obstacles.
Note that we provide three different control modes for you, ordered with increasing level of abstraction: commanding individual single-rotor thrusts (SRT), specifying mas-normalized collective thrust and bodyrates (CTBR), and outputting linear velocity commands and yawrate (LINVEL). The choice of control modality is up to you.
Overall, the more low-level you go, the more difficult is going to be to mantain stability, but the more agile your drone will be.

**Testing your approach in the simulator:**

Make sure you have completed the installation of the flight API before continuing.
To use the competition software, three steps are required:
1. Start the simulator
   ```
   roslaunch envsim visionenv_sim.launch render:=True
   # Using the GUI, press Arm & Start to take off.
   python evaluation_node.py
   ```
2. Start your user code. This code will generate control commands based on the sensory observations. You can toggle vision-based operation by providing the argument `--vision_based`.
   ```
   cd envtest/ros
   python run_competition.py [--vision_based]
   ```
3. Tell your code to start! Until you publish this message, your code will run but the commands will not be executed. We use this to ensure fair comparison between approaches as code startup times can vary, especially for learning-based approaches.
   ```
   rostopic pub /kingfisher/start_navigation std_msgs/Empty "{}" -1
   ```

TODO: we probably should prepare some bash scripts to automate parts of this...
TODO: we also need some automatic evaluation script, i.e. something that measures time and number of collisions or so
TODO: we need to provide a conda install file or a requirements.txt to install python dependencies.

## Training (Optional)
We also provide an easy interface for training your navigation policy using RL. While this is not required to compete, it could just make your job easier if you plan on using RL.

### Installation
Run the `setup_py.bash` in the main folder of this repository, it will ask for sudo permissions.

```bash
./setup_py.bash
```

### Usage

Follow [this guide](/envtest/python/README.md) to know more about how to use the training code.


## TODOs

[] Double check the command code, which has caused some issues
[] ROS launch file for RL policy

