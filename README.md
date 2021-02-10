# ROS-Docker
[![linux](https://github.com/autonomousrobotshq/ROS-Docker/workflows/Linux/badge.svg)](https://github.com/autonomousrobotshq/ROS-Docker/actions?workflow=Linux)
[![macos](https://github.com/autonomousrobotshq/ROS-Docker/workflows/MacOS/badge.svg)](https://github.com/autonomousrobotshq/ROS-Docker/actions?workflow=MacOS)
## Develop and test ROS in a Docker container

### Quickstart

Run:

    ros-docker.sh help

to display the following options:

```
./ros-docker.sh - Develop and test ROS in Docker
usage:
./ros-docker.sh [options] package [package directory] [shell command] -> for single package
./ros-docker.sh [options] workspace [catkin_ws directory] [shell command] -> for entire workspace
options:
         -h, --help -> will display these very words
         -c, --clean -> do a clean build (no preinstalled rosdeps)
         -r, --rosdep -> install rosdependency before execution of commands
         -R, --rebuild -> force rebuilding of Docker image
         -d, --dryrun -> do a dryrun (show commands that would be executed)
         -D, --debug -> internal debugging -> stop on first error
examples:
./ros-docker.sh (-c) (-r) (-d) (-R) package /opt/catkin_ws/src/[package dir] [command]
./ros-docker.sh (-c) (-r) (-d) (-R) workspace /opt/catkin_ws [command]
```

### Github Actions

To use this repository to run tests on your ROS package or ROS catkin workspace:

1. Select either 'action_package.yml' or 'action_workspace.yml' from this repository.
2. Copy this yaml file to your repository in a created subfolder '.github/workflows/'
3. Give it a good name. Most of the times 'Build.yml' will do.
4. Verify that the yaml as templated does as you require
5. Place a status badge in your README.md, like this:

```markdown
    [![build](https://github.com/[user]/[repo]/workflows/[workflow name]/badge.svg)](https://github.com/[user]/[repo]/actions?workflow=[workflow name])
```

#### Notes

* For a single ROS package/workspace in the root of your repository: action_package.yml' will work out-of-the-box

### Features

* images will be built automatically before entering into container
* provides quick and portable way to work with ROS
* provides easy way to do compilation/unittests for CI-purposes

### Problems

* right now this repo is hardcoded to test in Ubuntu:Focal with Ros:Noetic
