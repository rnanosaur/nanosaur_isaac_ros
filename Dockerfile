# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# Jetpack 4.6
FROM dustynv/ros:foxy-ros-base-l4t-r32.6.1

# Install OpenCV dependencies
RUN apt-get update && \
    apt-get install libjpeg-dev zlib1g-dev -y && \
    rm -rf /var/lib/apt/lists/*

########### INSTALL ISAAC ROS ###########

# Download and build nanosaur_isaac_ros
ENV ISAAC_ROS_WS /opt/isaac_ros_ws
RUN mkdir -p $ISAAC_ROS_WS/src
# Copy wstool isaac_ros.rosinstall
COPY nanosaur_isaac_ros/rosinstall/isaac_ros.rosinstall isaac_ros.rosinstall

# Initialize ROS2 workspace
RUN pip3 install wheel && \
    pip3 install -U wstool && \
    wstool init $ISAAC_ROS_WS/src && \
    wstool merge -t $ISAAC_ROS_WS/src isaac_ros.rosinstall && \
    wstool update -t $ISAAC_ROS_WS/src

ADD libs/vpi1/ /usr/share/vpi1/
ADD libs/nvidia/vpi1/ /opt/nvidia/vpi1/

# Load ROS2 sources
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd $ISAAC_ROS_WS && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# Remove VPI after build docker
RUN rm -R /usr/share/vpi1/
RUN rm -R /opt/nvidia/vpi1/

########### INSTALL NANOSAUR ISAAC ROS ###########

# Download and build nanosaur_isaac_ros
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src

ADD . $ROS_WS/src/nanosaur_isaac_ros
RUN rm -R $ROS_WS/src/nanosaur_isaac_ros/libs

# Load ROS2 sources
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    . $ISAAC_ROS_WS/install/setup.sh && \
    cd $ROS_WS && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Change workdir
WORKDIR $ROS_WS
# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
# run ros package launch file
# CMD ["ros2", "launch", "nanosaur_isaac_follower", "isaac_follower.launch.py"]

