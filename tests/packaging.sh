#!/bin/sh

# quit on first error
set -e

./ros-docker.sh --dryrun -r package ~/Projects/ros_packages "catkin_make"
./ros-docker.sh --dryrun -c -r package ~/Projects/ros_packages "catkin_make"
./ros-docker.sh --dryrun -c package ~/Projects/ros_packages "ls"
./ros-docker.sh --dryrun -d package ~/Projects/ros_packages "catkin_make"
