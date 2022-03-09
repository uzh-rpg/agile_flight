#!/bin/bash

# Pass number of rollouts as argument
if [ $1 ]
then
  N="$1"
else
  N=10
fi

# Set Flightmare Path if it is not set
if [ -z $FLIGHTMARE_PATH ]
then
  export FLIGHTMARE_PATH=$PWD/flightmare
fi

# Launch the simulator, unless it is already running
if [ -z $(pgrep visionsim_node) ]
then
  roslaunch envsim visionenv_sim.launch render:=True &
  ROS_PID="$!"
  echo $ROS_PID
  sleep 10
else
  ROS_PID=""
fi

SUMMARY_FILE="evaluation.yaml"
"" > $SUMMARY_FILE

# Perform N evaluation runs
for i in $(eval echo {1..$N})
do
  # Publish simulator reset
  rostopic pub /kingfisher/agiros_pilot/off std_msgs/Empty "{}" --once
  rostopic pub /kingfisher/agiros_pilot/reset_sim std_msgs/Empty "{}" --once
  rostopic pub /kingfisher/agiros_pilot/enable std_msgs/Bool "data: true" --once

  export ROLLOUT_NAME="rollout_""$i"
  echo "$ROLLOUT_NAME"

  cd ./envtest/ros/
  python evaluation_node.py &
  PY_PID="$!"
  cd -

  sleep 0.5
  rostopic pub /kingfisher/start_navigation std_msgs/Empty "{}" --once

  # Wait until the evaluation script has finished
  while ps -p $PY_PID > /dev/null
  do
    sleep 1
  done

  cat "$SUMMARY_FILE" "./envtest/ros/summary.yaml" > "tmp.yaml"
  mv "tmp.yaml" "$SUMMARY_FILE"
done


if [ $ROS_PID ]
then
  kill -SIGINT "$ROS_PID"
fi
