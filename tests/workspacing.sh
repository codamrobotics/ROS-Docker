#!/bin/sh

# quit on first error
set -e

./ros-docker.sh --dryrun -r workspace /tmp "catkin_make"
./ros-docker.sh --dryrun -c -r workspace /tmp "catkin_make"
./ros-docker.sh --dryrun -c workspace /tmp "ls"
./ros-docker.sh --dryrun -d workspace /tmp "catkin_make"
