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
	[ "${REALPATH: -1}" = "." ] && [ ! ${#REALPATH} -eq 1 ] && REALPATH=$(echo "$REALPATH" | sed 's/.$//')
	[ "${REALPATH: -1}" = "/" ] && [ ! ${#REALPATH} -eq 1 ] && REALPATH=$(echo "$REALPATH" | sed 's/\/$//')
	cd "$OURPWD"
	echo "$REALPATH"
} # End of realpath()


## environment variables
#
THIS="$(realpath $0)"
BASEDIR="$(realpath "$(dirname "$0")")"
STAGING="$BASEDIR/build_staging"
LIB="$BASEDIR/lib"

## includes
#
# shellcheck source=lib/colours.sh
source "$LIB"/colours.sh

## setup clean up traps 
#
clean_up()
{
    [  -d "$STAGING" ] && rm -rf "$STAGING"
}

trap "clean_up INT" INT
trap "clean_up TERM" TERM
trap "clean_up EXIT" EXIT

#
## Sanitize input
# Sna-ke becomes sna_ke
# reads from stdin or arguments list
sanitize() {
	[ $# -eq 0 ] && read input || input="$*"
	echo $input | sed 's/-/_/g' | tr -dc '[:alnum:]_\n\r' | tr '[:upper:]' '[:lower:]'
} # End of sanitize()

#
## check a condition and quit if it is false
#
assert() {
	[ $# -lt 2 ] && { echo "assert called without condition + linenumber!"; exit 1; }
	(eval "$1" &>/dev/null) || { echo -e "${COLOR_RED}assertion failed!${COLOR_NC} : $*"; usage; exit 1; }
}

#
## install bindings
#
install() {
	local alias="alias ros='$THIS'"
	local rcs=("$HOME/.zshrc" "$HOME/.bashrc" )
	for rc in "${rcs[@]}"; do
		rc=$(realpath $rc)
		local _op="echo $alias >> $rc"
		[ -f $rc ] && [[ "$(grep -e '^alias ros=' $rc )" == "" ]] && { [ -z "${DRYRUN+x}" ] && eval "$_op" || dryrun @ $LINENO: "$_op"; }
	done
} # end of install

#
## Display usage options
#
usage() {
	cat<<-EOF
	$0 - Develop and test ROS in Docker
	usage:
	$0 [options] package [package directory] [shell command] -> for single package
	$0 [options] workspace [catkin_ws directory] [shell command] -> for entire workspace
	options:
	         -h, --help -> will display these very words
	         -i, --install -> will add alias 'ros' to ~/.bashrc and ~/.zshrc
	         -c, --clean -> do a clean build (no preinstalled rosdeps)
	         -r, --rosdep -> install rosdependency before execution of commands
	         -b, --rebuild -> force rebuilding of Docker image
	         -d, --dryrun -> do a dryrun (show commands that would be executed)
	         -D, --debug -> internal debugging -> stop on first error
	examples:
	$0 (-c) (-r) (-d) (-b) package /opt/catkin_ws/src/[package dir] [command]
	$0 (-c) (-r) (-d) (-b) workspace /opt/catkin_ws [command]
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
			-h|--help)		usage;				exit 0; ;;
			-i|--install)	export TARGET=install;		shift; ;;
			-c|--clean)		export FLAVOUR=CLEAN;		shift; ;;
			-r|--rosdep)	export ROSDEP=ROSDEP;		shift; ;;
			-b|--rebuild)	export FORCE_REBUILD=TRUE; shift; ;;
			-d|--dryrun)	export DRYRUN=TRUE;		shift; ;;
			-D|--debug)		export DEBUG=TRUE; set -e;	shift; ;;
			package)		export TARGET=PACKAGE;		shift; ;;
			workspace)		export TARGET=WORKSPACE;	shift; ;;
			*)
				# read directory or command
				if [ -d $1 ]; then
					LPATH=$(realpath $1)
				elif [ ! -z ${TARGET+x} ] && [ ! -z ${LPATH} ]; then
					CMD="$*"; break;
				else
					usage; exit 1
				fi

			shift; ;;
		esac
	done

	# catch install target
	[[ "$TARGET" == "install" ]] && { install; exit 0; }

	# these values must be set
	assert '[ ! -z ${LPATH+x} ] && [ -d ${LPATH} ]' $LINENO : "No path specified!"
	assert '[ ! -z ${FLAVOUR+x} ]' $LINENO
	assert '[ ! -z ${TARGET+x} ]' $LINENO : "Must specify: workspace / package!"
} # End of read_input()

dryrun() {
	echo -e "${COLOR_YELLOW}DRYRUN${COLOR_NC} @ $*"
} # End of drurun()

notify() {
	echo -e "${COLOR_GREEN}STATUS${COLOR_NC} @ $*"
} # End of drurun()

#
## Build the Docker image
#
build() {
	local catkin_ws=/opt/catkin_ws

	# setup empty stub folder
	mkdir $STAGING

	case ${FLAVOUR} in
		CLEAN)
			local project="clean"
		;;
		TARGETED)
			local project;
			project="$(sanitize "$(echo "$LPATH" | rev | cut -f1 -d/ | rev)")"

			# copy over files from project to temporarily insert into docker container for rosdep
			local _op="cp -a $LPATH $STAGING/"
			if [ -z "${DRYRUN+x}" ]; then
				eval "$_op" || exit 1
			else
				dryrun $LINENO: "$_op"
			fi
		 ;;
	esac
	
	assert '[ ! -z ${project+x} ]' $LINENO
	IMAGE=ros-docker-$project
	IMAGE_TAG=0.1

	echo "Docker building image \"$IMAGE:$IMAGE_TAG\"..."
	local _op="docker build $BASEDIR -t $IMAGE:$IMAGE_TAG"
	if [ -z ${DRYRUN+x} ]; then
		if [ ! -z ${FORCE_REBUILD} ] || [[ "$(docker images -q $IMAGE:$IMAGE_TAG 2>/dev/null)" == "" ]]; then
            if [[ ! "$(docker images -q ros:noetic-ros-core 2> /dev/null)" == "" ]]; then
                # TODO: HARCODED for noetic
			    notify $LINENO: "Image ros:noetic-ros-core already exists. Trying to pull latest.."
                docker pull ros:noetic-ros-core
            fi
			eval "$_op" || exit 1
		else
			notify $LINENO: "Image $IMAGE:$IMAGE_TAG already exists. Continuing.."
		fi
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
			;;
		WORKSPACE)
			RPATH="$catkin_ws"
			;;
	esac
	
	assert '[ ! -z ${LPATH+x} ]' $LINENO
	assert '[ ! -z ${RPATH+x} ]' $LINENO
	assert '[ ! -z ${IMAGE+x} ]' $LINENO
	assert '[ ! -z ${IMAGE_TAG+x} ]' $LINENO
	assert '[ ! -z ${CMD+x} ]' $LINENO "No command specified -> Try appending 'bash' as argument."
	assert '[ ! -z ${ROSDEP+x} ]' $LINENO

	echo "Docker shadowing image \"$IMAGE:$IMAGE_TAG\"..."
	#if [[ $- == *i* ]]; then
	if [ -t 0 ]; then
		# running interactively
		local _op="docker run -it --rm --init -v "$LPATH:$RPATH" $IMAGE:$IMAGE_TAG $ROSDEP sh -c $CMD"
	else
		# running headless
Related JIRA tickets or pull requests.
		local _op="docker run --rm --init -v "$LPATH:$RPATH" $IMAGE:$IMAGE_TAG $ROSDEP sh -c $CMD"
	fi

	if [ -z ${DRYRUN+x} ]; then
		eval "$_op" || exit 
	else
		dryrun "$_op"
	fi
} # End of run()

read_input "$@" || exit 1
build || exit 1
run || exit 1
