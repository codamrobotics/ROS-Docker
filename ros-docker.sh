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

basedir=$(realpath $(dirname "$0"))
lib=$basedir/lib

# assume current working directory if no argument is specified
if [ $# -eq 0 ]; then
	lpath=$(realpath $PWD)
	project=$(echo $(realpath $PWD) | rev | cut -f1 -d/ | rev)
	cmd='exit'
# first argument is the target directory
elif [ $# -eq 1 ] && [ -d $1 ] ; then
	lpath=$(realpath $1)
	project=$(echo $(realpath $1) | rev | cut -f1 -d/ | rev)
	cmd='exit'
# first argument is clean
elif [ $# -eq 1 ] && [ "$1" == "clean" ] ; then
	lpath=$(realpath $PWD)
	cmd='exit'
# first argument is clean and second argument is target directory
elif [ $# -eq 2 ] && [ "$1" == "clean" ] && [ -d $2 ]; then
	lpath=$(realpath $2)
	cmd='exit'
# first argument is clean, second argument is target directory, third+ is bash command
elif [ $# -gt 2 ] && [ "$1" == "clean" ] && [ -d $2 ]; then
	lpath=$(realpath $2)
	cmd="${@:3}"
# first argument is clean, second+ is bash command
elif [ $# -gt 1 ] && [ "$1" == "clean" ]; then
	lpath=$(realpath $PWD)
	cmd="${@:2}"
# second+ argument is a bash command
elif [ $# -gt 1 ] && [ -d $1 ]; then
	lpath=$(realpath $1)
	project=$(echo $(realpath $1) | rev | cut -f1 -d/ | rev)
	cmd="${@:2}"
else
	cat<<-EOF
	$0 usage:

	   With pre-installed ros dependencies:
	   $0                                       -> compiles current directory, exits on completion
	   $0 [target folder]                       -> compiles [target folder], exits on completion
	   $0 [target folder] [shell command]       -> compiles [target folder], runs [shell command] on completion

	   Without pre-installed ros dependencies:
	   $0 clean                                 -> compiles current directory, exits on completion
	   $0 clean [target folder]                 -> compiles [target folder], exits on completion
	   $0 clean [target folder] [shell command] -> compiles [target folder], runs [shell command] on completion

	   $0 help                                  -> will display these fine words of help
	examples:
	   $0 ~/Projects/Spine--ROS-Messages bash   -> build with deps and will attempt compilation of ~/Projects/[project] and run drop you in shell afterwards
	   $0 clean ~/Projects/[project] bash       -> build clean ros-core and will attempt compilation of ~/Projects/[project] and run drop you in shell afterwards
	EOF
	exit 1
fi

version=0.1
catkin_ws=/opt/catkin_ws
if [ ! -z "$project" ]; then
	# build & run for
	$lib/ros-docker-build.sh $lpath \
	&& docker run -it --rm --init -v "$lpath:/src" ros-docker-$project:$version sh -c "cd $catkin_ws/src/build; $cmd"
else
	# build & run clean
	$lib/ros-docker-build.sh \
	&& docker run -it --rm --init -v "$lpath:/src" ros-docker-clean:$version sh -c "cd $catkin_ws/src/build; $cmd"
fi

