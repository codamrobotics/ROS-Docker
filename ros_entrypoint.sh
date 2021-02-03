#!/bin/bash
#
# Spine - Spine - MCU code for robotics.
# Copyright (C) 2019-2021 Codam Robotics
#
# This file is part of Spine.
#
# Spine is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Spine is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Spine.  If not, see <http://www.gnu.org/licenses/>.
#
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

catkin_ws=/opt/catkin_ws
src=/src

# copy over src files
size=$(du -bs $src | cut -f1)
if [ $size -gt 1000000000 ]; then
	echo "Target directory is bigger than 1000M! Aborting."
	exit 1
fi

cp -r $src $catkin_ws/src/build

# setup catkin
cd $catkin_ws/src && catkin_init_workspace

# setup dependencies
if ! rosdep check --from-paths $catkin_ws/src --ignore-src -r -y; then
	apt update
	rosdep update
	rosdep install --from-paths $catkin_ws/src --ignore-src -r -y
fi

# make catkin_ws
cd $catkin_ws && catkin_make

#only execute shell if there is a failure
if [ ! $? -eq 0 ]; then
	exec "bash"
else
	exec "$@"
fi
