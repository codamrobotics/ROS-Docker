#!/bin/sh
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

#http://stackoverflow.com/a/18443300/441757
function realpath() {
	OURPWD=$PWD
	cd "$(dirname "$1")"
	LINK=$(readlink "$(basename "$1")")
	while [ "$LINK" ]; do
	  cd "$(dirname "$LINK")"
	  LINK=$(readlink "$(basename "$1")")
	done
	REALPATH="$PWD/$(basename "$1")"
	[ "${REALPATH: -1}" = "." ] && [ ! ${#REALPATH} -eq 1 ] && REALPATH=$(echo $REALPATH | sed 's/.$//')
	[ "${REALPATH: -1}" = "/" ] && [ ! ${#REALPATH} -eq 1 ] && REALPATH=$(echo $REALPATH | sed 's/\/$//')
	cd "$OURPWD"
	echo "$REALPATH"
}

# assume current working directory if no argument is specified
if [ $# -eq 0 ]; then
	lpath=$PWD
	cmd='exit'
# first argument is the target directory
elif [ $# -eq 1 ] && [ -d $1 ] ; then
	lpath=$(realpath $1)
	cmd='exit'
# second argument is a bash command
elif [ $# -gt 1 ] && [ -d $1 ] ; then
	lpath=$(realpath $1)
	cmd="${@:2}"
else
	cat<<-EOF
	$0 usage:
	   $0                                 -> compiles current directory, exits on completion
	   $0 [target folder]                 -> compiles [target folder], exits on completion
	   $0 [target folder] [shell command] -> compiles [target folder], runs [shell command] on completion
	examples:
	   $0 ~/Projects/Spine--ROS-Messages bash -> will attempt compilation of ~/Projects/Spine--ROS-Messages and run drop you in shell afterwards
	EOF
	exit 1
fi

catkin_ws=/opt/catkin_ws
docker run -it --rm --init -v "$lpath:/src" ros-docker:0.1 sh -c "cd $catkin_ws/src/build; $cmd"
