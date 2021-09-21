#!/bin/bash
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

bold=`tput bold`
red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
reset=`tput sgr0`

PLATFORM="$(uname -m)"

if [[ $PLATFORM != "aarch64" ]]; then
    echo "${red}Run this script only on ${bold}${green}NVIDIA${reset}${red} Jetson platform${reset}"
    exit 33
fi

#Workaround build with VPI
if [ ! -d libs/nvidia/ ] ; then
    echo "${bold}[Workaround]${reset} local copy VPI library"
    mkdir -p libs/nvidia/
    cp -R /usr/share/vpi1/ libs/vpi1
    cp -R /opt/nvidia/vpi1/ libs/nvidia/vpi1/
fi

# Build docker ISAAC GEMS for ROS
echo "Build ISAAC GEMs for ROS"
docker build -t nanosaur/nanosaur_isaac_ros:latest .