#!/bin/sh

# quit on first error
set -e

./ros-docker.sh --help
./ros-docker.sh -h

./ros-docker.sh --dryrun --install
./ros-docker.sh --dryrun -i
