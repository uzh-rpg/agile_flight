# Agile Flight

Agile Flight is a super awesome research project developed by researchers from Robotics and Perception Group. 
The goal is to build and benchmark autonomous algorithms for vision-based agile flight.


## Installation with ROS

Create a new catkin workspace. 
```
TODO

cd PATH_TO_YOUR_CATKIN_WS/src
git clone git@github.com:uzh-rpg/agile_flight.git
cd agile_flight
```

Run the `setup_ros.bash` in the main folder of this repository, it will ask for sudo permissions.

```bash
./setup_ros.bash
```

## Installation with Python 
Run the `setup_py.bash` in the main folder of this repository, it will ask for sudo permissions.

```bash
./setup_py.bash
```
## Usage with ROS
Start the simulation:
```
roslaunch envsim visionenv_sim.launch render:=True use_unity:=True
```

## Usage with Python 
Start the simulation:
```
python3 -m envpy.run_vision_demo --render 1
```
