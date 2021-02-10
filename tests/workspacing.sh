#!/bin/sh

# quit on first error
set -e

./ros-docker.sh --dryrun -r workspace /opt/ros/catkin_ws "catkin_make"
./ros-docker.sh --dryrun -c -r workspace /opt/ros/catkin_ws "catkin_make"
./ros-docker.sh --dryrun -c workspace /opt/ros/catkin_ws "ls"
./ros-docker.sh --dryrun -d workspace /opt/ros/catkin_ws "catkin_make"
