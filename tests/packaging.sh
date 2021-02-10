#!/bin/sh

# quit on first error
set -e

./ros-docker.sh --dryrun -r package /tmp "catkin_make"
./ros-docker.sh --dryrun -c -r package /tmp "catkin_make"
./ros-docker.sh --dryrun -c package /tmp "ls"
./ros-docker.sh --dryrun -d package /tmp "catkin_make"
