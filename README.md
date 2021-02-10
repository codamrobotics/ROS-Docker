# ROS-Docker
[![linux](https://github.com/autonomousrobotshq/ROS-Docker/workflows/Linux/badge.svg)](https://github.com/autonomousrobotshq/ROS-Docker/actions?workflow=Linux)
[![macos](https://github.com/autonomousrobotshq/ROS-Docker/workflows/MacOS/badge.svg)](https://github.com/autonomousrobotshq/ROS-Docker/actions?workflow=MacOS)

### Quickstart

run:

    ros-docker.sh help

to display the following options:

    ./ros-docker.sh - Develop and test ROS in Docker
    usage:
    ./ros-docker.sh [options] package [package directory] [shell command] -> for single package
    ./ros-docker.sh [options] workspace [catkin_ws directory] [shell command] -> for entire workspace
    ./ros-docker.sh help -> will display these very words
    options:
             -c, --clean  -> do a clean build (no preinstalled rosdeps)
             -r, --rosdep -> install rosdependency before execution of commands
             -d, --dryrun -> do a dryrun (show commands that would be executed)
             -D, --debug -> internal debugging -> stop on first error
    examples:
    ./ros-docker.sh (-c) (-r) (-d) package /opt/catkin_ws/src/[package dir] [command]
    ./ros-docker.sh (-c) (-r) (-d) workspace /opt/catkin_ws [command

### Features

* images will be build automaticly before entering into container
