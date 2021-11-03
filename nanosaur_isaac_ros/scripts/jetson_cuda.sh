#!/bin/bash
# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

if [ $# -le 2 ] ; then
    echo "No argument supplied"
    echo "Please add these arguments: CUDA L4T_RELEASE TENSORRT"
    exit 1
fi

# Install CUDA
# https://gitlab.com/nvidia/container-images/l4t-base/-/blob/master/Dockerfile.cuda
CUDA=$1
RELEASE=$2
TENSORRT=$3
ROS_DISTRO=$4
ROS_WS=$5
ISAAC_ROS_WS=$6
echo "CUDA=$CUDA L4T=$RELEASE TENSORRT=$TENSORRT ROS_DISTRO=$ROS_DISTRO ROS_WS=$ROS_WS ISAAC_ROS_WS=$ISAAC_ROS_WS"

if [ ! -f /usr/local/cuda/version.txt ]; then
    INSTALL_CUDA=true
else
    INSTALL_CUDA=false
fi

# Check if CUDA is installed
if $INSTALL_CUDA ; then
    echo "Install CUDA"
    # Install CUDA
    apt-get update && apt-get install -y --no-install-recommends gnupg2 ca-certificates
    # COPY jetson-ota-public.key /etc/jetson-ota-public.key
    curl https://gitlab.com/nvidia/container-images/l4t-base/-/raw/master/jetson-ota-public.key -o /etc/jetson-ota-public.key
    apt-key add /etc/jetson-ota-public.key
    echo "deb https://repo.download.nvidia.com/jetson/common $RELEASE main" >> /etc/apt/sources.list

    CUDAPKG=$(echo $CUDA | sed 's/\./-/');

    apt-get update
    apt-get install -y --no-install-recommends \
        cuda-libraries-$CUDAPKG \
        cuda-nvtx-$CUDAPKG \
        cuda-libraries-dev-$CUDAPKG \
        cuda-minimal-build-$CUDAPKG \
        cuda-license-$CUDAPKG \
        cuda-command-line-tools-$CUDAPKG

    # Link CUDA library
    ln -s /usr/local/cuda-$CUDA /usr/local/cuda
fi

echo "Install VPI"
# Install libnvvpi1 for Isaac ROS
apt-get update && apt-get install -y --no-install-recommends libnvvpi1 vpi1-dev

# Load ROS2 sources
. /opt/ros/$ROS_DISTRO/install/setup.sh

exit 0 # <----------------------------------------------

# Load ROS2 sources
cd $ISAAC_ROS_WS
echo "BUILD FOLDER $ISAAC_ROS_WS"
colcon build --symlink-install \
    --packages-select isaac_ros_apriltag_interfaces isaac_ros_apriltag nanosaur_msgs isaac_ros_image_proc isaac_ros_common isaac_ros_test \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release || { echo "ros2 Isaac ROS build failure!"; exit 1; }

. $ISAAC_ROS_WS/install/setup.sh

# Build ros_ws
cd $ROS_WS
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release || { echo "ros2 build failure!"; exit 1; }

if $INSTALL_CUDA ; then
    echo "Remove CUDA"
    # Remove CUDA
    apt-get purge -y \
        cuda-libraries-$CUDAPKG \
        cuda-nvtx-$CUDAPKG \
        cuda-libraries-dev-$CUDAPKG \
        cuda-minimal-build-$CUDAPKG \
        cuda-license-$CUDAPKG \
        cuda-command-line-tools-$CUDAPKG
    apt-get autoremove -y

fi

echo "Remove VPI"
# Remove VPI1
apt-get purge -y libnvvpi1 vpi1-dev
apt-get autoremove -y

echo "Clean APT build"
# Clean apt build
rm -rf /var/lib/apt/lists/*
