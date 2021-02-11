#!/bin/sh

# quit on first error
set -e

./ros-docker.sh --dryrun package /tmp "catkin_make"
./ros-docker.sh --dryrun --rosdep package /tmp "catkin_make"
./ros-docker.sh --dryrun --rosdep --rebuild package /tmp "catkin_make"
./ros-docker.sh --dryrun --rosdep --rebuild --debug package /tmp "catkin_make"

./ros-docker.sh --dryrun --clean package /tmp "catkin_make"
./ros-docker.sh --dryrun --clean --rosdep package /tmp "catkin_make"
./ros-docker.sh --dryrun --clean --rosdep --rebuild package /tmp "catkin_make"
./ros-docker.sh --dryrun --clean --rosdep --rebuild --debug package /tmp "catkin_make"

./ros-docker.sh -d package /tmp "catkin_make"
./ros-docker.sh -d -r package /tmp "catkin_make"
./ros-docker.sh -d -r -b package /tmp "catkin_make"
./ros-docker.sh -d -r -b -D package /tmp "catkin_make"

./ros-docker.sh -d -c package /tmp "catkin_make"
./ros-docker.sh -d -c -r package /tmp "catkin_make"
./ros-docker.sh -d -c -r -b package /tmp "catkin_make"
./ros-docker.sh -d -c -r -b -D package /tmp "catkin_make"
