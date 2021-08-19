#!/bin/bash

#Workaround build with VPI
mkdir -p libs/nvidia/
cp -R /usr/share/vpi1/ libs/vpi1
cp -R /opt/nvidia/vpi1/ libs/nvidia/vpi1/

# Build docker ISAAC ROS
docker build -t nanosaur/nanosaur_isaac_ros:latest .