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

FROM ros:noetic-ros-core

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
	build-essential \
	cmake \
	git \
	vim \
	ranger \
	openssh-server \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
	python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# setup catkin
RUN mkdir -p /opt/catkin_ws/src

# setup rosdep
RUN rosdep init # && rosdep update

# install ros dependencies if required
COPY build_staging* /opt/catkin_ws/src/target

RUN if [ ! -z "$(ls -A /opt/catkin_ws/src)" ]; then \
	find /opt/catkin_ws/src  -type d \( -name "build" -o -name "devel" -o -name "install" \) -print0 | xargs -r0 -- rm -r \
	&& apt update \
	&& rosdep update \
	&& rosdep install --from-paths /opt/catkin_ws/src --ignore-src -r -y \
	&& rm -rf /opt/catkin_ws/src/* \
    && rm -rf /var/lib/apt/lists/*; \
fi

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
