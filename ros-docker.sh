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

#
### ros-docker.sh
# Build a ROS build container for development and testing
# Checkout the README.md or run 'ros-docker.sh help' to figure out how to get started
#

## Core dependency: Returns absolute path of a folder
# source: http://stackoverflow.com/a/18443300/441757
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
} # End of realpath()


## environment variables
# 
basedir=$(realpath "$(dirname "$0")")
lib=$basedir/lib
build_tmp=$lib/build_tmp

## includes
#
source $lib/colours.sh

## setup clean up traps 
#
clean_up()
{
    [  -d "$build_tmp" ] && rm -rf "$build_tmp"
}

trap "clean_up INT" INT
trap "clean_up TERM" TERM
trap "clean_up EXIT" EXIT

#
## Sanitize input
# Sna-ke becomes sna_ke
# reads from stdin or arguments list
sanitize() {
	[ $# -eq 0 ] && read input || input="$@"
	echo $input | sed 's/-/_/g' | tr -dc '[:alnum:]_\n\r' | tr '[:upper:]' '[:lower:]'
} # End of sanitize()

#
## check a condition and quit if it is false
#
assert() {
	[ $# -lt 2 ] && { echo "assert called without condition + linenumber!"; exit 1; }
	(eval "$1" &>/dev/null) || { echo -e "${COLOR_RED}assertion failed!${COLOR_NC} : $@"; usage; exit 1; }
}

#
## Display usage options
#
usage() {
	cat<<-EOF
	$0 - Develop and test ROS in Docker
	usage:
	$0 [options] package [package directory] [shell command] -> for single package
	$0 [options] workspace [catkin_ws directory] [shell command] -> for entire workspace
	$0 help -> will display these very words
	options:
	         -c, --clean  -> do a clean build (no preinstalled rosdeps)
	         -r, --rosdep -> install rosdependency before execution of commands
	         -d, --dryrun -> do a dryrun (show commands that would be executed)
	         -D, --debug -> internal debugging -> stop on first error
	examples:
	$0 (-c) (-r) (-d) package /opt/catkin_ws/src/[package dir] [command]
	$0 (-c) (-r) (-d) workspace /opt/catkin_ws [command]
	EOF
} # End of usage()

#
## Read input arguments given on the commandline
# 
read_input() {
	# set defaults
	FLAVOUR=TARGETED
	ROSDEP=NOROSDEP

	# display usage if no arguments were given
	[ $# -eq 0 ] && { usage; exit 1; }

	while [ $# -gt 0 ]; do
		case "$1" in
			-h|--help)		usage;				exit 1; ;;
			-c|--clean)		FLAVOUR=CLEAN		shift; ;;
			-r|--rosdep)	ROSDEP=ROSDEP		shift; ;;
			-d|--dryrun)	DRYRUN=TRUE			shift; ;;
			-D|--debug)		DEBUG=TRUE; set -e;	shift; ;;
			package)		TARGET=PACKAGE		shift; ;;
			workspace)		TARGET=WORKSPACE	shift; ;;
			*)
				# read directory or command
				if [ -d $1 ]; then
					LPATH=$(realpath $1)
				elif [ ! -z ${TARGET+x} ] && [ ! -z ${LPATH} ]; then
					CMD="$*"
					break;
				else
					usage
					exit 1
				fi
			shift; ;;
		esac
	done

	# these values must be set
	assert '[ ! -z ${LPATH+x} ] && [ -d ${LPATH} ]' $LINENO : "No path specified!"
	assert '[ ! -z ${FLAVOUR+x} ]' $LINENO
	assert '[ ! -z ${TARGET+x} ]' $LINENO : "Must specify: workspace / package!"
} # End of read_input()

dryrun() {
	echo -e "${COLOR_YELLOW}DRYRUN${COLOR_NC} @ $LINENO : $@"
} # End of drurun()

#
## Build the Docker image
#
build() {
	local version=0.1
	local catkin_ws=/opt/catkin_ws

	# setup empty stub folder
	mkdir $build_tmp

	case ${FLAVOUR} in
		CLEAN)
			local project="clean"
		break; ;;
		TARGETED)
			local project="$(echo $LPATH | rev | cut -f1 -d/ | rev | sanitize)"

			# copy over files from project to temporarily insert into docker container for rosdep
			local _op="cp -r $LPATH $build_tmp/"
			if [ -z "${DRYRUN+x}" ]; then
				eval "$_op" || exit 1
			else
				dryrun $LINENO: "$_op"
			fi
		break; ;;
	esac
	
	assert '[ ! -z ${project+x} ]' $LINENO
	IMAGE=ros-docker-$project
	IMAGE_VERSION=0.1

	echo "Docker building image \"$IMAGE:$IMAGE_VERSION\"..."
	local _op="docker build $lib -t $IMAGE:$IMAGE_VERSION"
	if [ -z ${DRYRUN+x} ]; then
		eval "$_op" || exit 1
	else
		dryrun $LINENO: "$_op"
	fi
} # End of build()

#
## Run the Docker image
#
run() {
	local catkin_ws=/opt/catkin_ws

	case ${TARGET} in
		PACKAGE)
			RPATH="$catkin_ws/src/$(echo $LPATH | rev | cut -f1 -d/ | rev)"
			break; ;;
		WORKSPACE)
			RPATH="$catkin_ws"
			break; ;;
	esac
	
	assert '[ ! -z ${LPATH+x} ]' $LINENO
	assert '[ ! -z ${RPATH+x} ]' $LINENO
	assert '[ ! -z ${IMAGE+x} ]' $LINENO
	assert '[ ! -z ${IMAGE_VERSION+x} ]' $LINENO
	assert '[ ! -z ${CMD+x} ]' $LINENO "No command specified -> Try appending 'bash' as argument."
	assert '[ ! -z ${ROSDEP+x} ]' $LINENO

	echo "Docker shadowing image \"$IMAGE:$IMAGE_VERSION\"..."
	local _op="docker run -it --rm --init -v "$LPATH:$RPATH" $IMAGE:$IMAGE_VERSION $ROSDEP sh -c "$CMD""
	if [ -z ${DRYRUN+x} ]; then
		eval "$_op" || exit 
	else
		dryrun "$_op"
	fi
} # End of run()

read_input "$@" || exit 1
build || exit 1
run || exit 1
