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
./ros-docker.sh -d -r -R package /tmp "catkin_make"
./ros-docker.sh -d -r -R -D package /tmp "catkin_make"

./ros-docker.sh -d -c package /tmp "catkin_make"
./ros-docker.sh -d -c -r package /tmp "catkin_make"
./ros-docker.sh -d -c -r -R package /tmp "catkin_make"
./ros-docker.sh -d -c -r -R -D package /tmp "catkin_make"
