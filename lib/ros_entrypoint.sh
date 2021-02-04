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

# bail on first error
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

catkin_ws=/opt/catkin_ws

# setup catkin
cd $catkin_ws/src && [ ! -f $catkin_ws/src/CMakeLists.txt ] && catkin_init_workspace || true

# setup dependencies
if [[ "$1" == "ROSDEP" ]] &&  ! rosdep check --from-paths $catkin_ws/src --ignore-src -r -y; then
	apt update
	rosdep update
	rosdep install --from-paths $catkin_ws/src --ignore-src -r -y
fi

# enter workspace
cd $catkin_ws

# execute shell commands
exec "${@:2}"
