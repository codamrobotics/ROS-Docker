#!/bin/sh

# quit on first error
set -e

./ros-docker.sh --dryrun workspace /tmp "catkin_make"
./ros-docker.sh --dryrun --rosdep workspace /tmp "catkin_make"
./ros-docker.sh --dryrun --rosdep --rebuild workspace /tmp "catkin_make"
./ros-docker.sh --dryrun --rosdep --rebuild --debug workspace /tmp "catkin_make"

./ros-docker.sh --dryrun --clean workspace /tmp "catkin_make"
./ros-docker.sh --dryrun --clean --rosdep workspace /tmp "catkin_make"
./ros-docker.sh --dryrun --clean --rosdep --rebuild workspace /tmp "catkin_make"
./ros-docker.sh --dryrun --clean --rosdep --rebuild --debug workspace /tmp "catkin_make"

./ros-docker.sh -d workspace /tmp "catkin_make"
./ros-docker.sh -d -r workspace /tmp "catkin_make"
./ros-docker.sh -d -r -R workspace /tmp "catkin_make"
./ros-docker.sh -d -r -R -D workspace /tmp "catkin_make"

./ros-docker.sh -d -c workspace /tmp "catkin_make"
./ros-docker.sh -d -c -r workspace /tmp "catkin_make"
./ros-docker.sh -d -c -r -R workspace /tmp "catkin_make"
./ros-docker.sh -d -c -r -R -D workspace /tmp "catkin_make"
