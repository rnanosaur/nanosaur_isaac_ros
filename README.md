# 🍏 nanosaur_isaac_ros

Nanosaur ISAAC ROS integration

# Develop

Test build
```
docker build -t nanosaur/nanoasur_isaac_ros:latest .
```

Run docker
```
docker run --rm -it --network host -v /usr/share/vpi1:/usr/share/vpi1 nanosaur/nanoasur_isaac_ros:latest bash
```

# reference

* [ISAAC ROS common](https://github.com/NVIDIA-AI-IOT/isaac_ros_common.git)
* [ISAAC ROS image pipeline](https://github.com/NVIDIA-AI-IOT/isaac_ros_image_pipeline.git)