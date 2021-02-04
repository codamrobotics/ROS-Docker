# ROS-Docker

### Usage

run:

    ros-docker.sh help

to display options.

* images will be built automaticly before entering into container

### Current Features

* ros-docker.sh [no clean] will take your project folder (or current folder) and install all ros dependencies listed in the package CmakeLists.txt (good for debugging)
* ros-docker.sh clean will built clean image and install dependencies on starting the container (good for testing entire package)

### Planned Features

* option to enter to container and work from there

Better documentation will follow soon!
