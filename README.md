# 🍏 nanosaur_isaac_ros

Nanosaur ISAAC ROS integration

# Develop

Build docker
```
. nanosaur_isaac_ros/scripts/build_isaac_ros.sh 
```

Run docker
```
docker run --rm -it --network host -v /usr/share/vpi1:/usr/share/vpi1 -v /opt/nvidia:/opt/nvidia nanosaur/follower:latest
```

Edit docker
```
docker run --rm -it --network host -v /usr/share/vpi1:/usr/share/vpi1 -v /opt/nvidia:/opt/nvidia -v $HOME/nanosaur_isaac_ros:/opt/ros_ws/src/nanosaur_isaac_ros nanosaur/follower:latest bash
```

## CPU version

```
docker build -t nanosaur/follower:cpu -f Dockerfile.cpu .
```

```
docker run --rm -it --network host nanosaur/follower:cpu
```

## Rebuild packages

Rebuild all packages: `colcon build --symlink-install`

## Publish

Remember to login before run this script!
```
. nanosaur_isaac_ros/scripts/push_isaac_ros.sh 
```

# reference

* [ISAAC ROS apriltag](https://github.com/NVIDIA-AI-IOT/isaac_ros_apriltag.git)
* [ISAAC ROS common](https://github.com/NVIDIA-AI-IOT/isaac_ros_common.git)
* [ISAAC ROS image pipeline](https://github.com/NVIDIA-AI-IOT/isaac_ros_image_pipeline.git)
* Add VPI for build on Jetson https://forums.developer.nvidia.com/t/vpi-installation-error-in-docker-container/160254/3