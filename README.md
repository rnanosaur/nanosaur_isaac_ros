# 🍏 nanosaur_isaac_ros

Nanosaur ISAAC ROS integration

# Develop

Build docker
```
./isaac_ros_build.sh
```

Run docker
```
docker run --rm -it --network host -v /usr/share/vpi1:/usr/share/vpi1 -v /opt/nvidia:/opt/nvidia nanosaur/nanosaur_isaac_ros:latest bash
```

Edit docker
```
docker run --rm -it --network host -v /usr/share/vpi1:/usr/share/vpi1 -v /opt/nvidia:/opt/nvidia -v $HOME/nanosaur_isaac_ros:/opt/ros_ws/src nanosaur/nanosaur_isaac_ros:latest bash
```

## Rebuild packages

Rebuild all packages: `colcon build --symlink-install`

# reference

* [ISAAC ROS common](https://github.com/NVIDIA-AI-IOT/isaac_ros_common.git)
* [ISAAC ROS image pipeline](https://github.com/NVIDIA-AI-IOT/isaac_ros_image_pipeline.git)
* Add VPI for build on Jetson https://forums.developer.nvidia.com/t/vpi-installation-error-in-docker-container/160254/3