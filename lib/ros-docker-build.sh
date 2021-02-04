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
realpath() {
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

# setup clean up traps
function clean_up()
{
    [  -d $basedir/build_tmp ] && rm -r $basedir/build_tmp
}
trap "clean_up INT" INT
trap "clean_up TERM" TERM
trap "clean_up EXIT" EXIT

# setup empty stub folder
mkdir $basedir/build_tmp

if [ $# -eq 1 ] || [ ! -d $1 ]; then
	path=$1
	project=$(echo $path | rev | cut -f1 -d/ | rev)

	# copy over files from project to temporarily insert into docker container for rosdep
	cp -r $path $basedir/build_tmp/
else
	project='clean'
fi

name=ros-docker-$project
version=0.1

docker build $basedir -t $name:$version || exit 1

rm -rf $basedir/build_tmp
