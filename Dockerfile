# Jetpack 4.6
FROM dustynv/ros:foxy-ros-base-l4t-r32.6.1

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Fundamentals
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    sudo \
    unzip \
    vim \
    wget \
    software-properties-common \
&& rm -rf /var/lib/apt/lists/*

# Install OpenCV dependencies
RUN apt-get update && apt-get install -y \
    libavformat-dev \
    libjpeg-dev \
    libopenjp2-7-dev \
    libpng-dev \
    libpq-dev \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libtiff-dev \
    pkg-config \
    yasm \
&& rm -rf /var/lib/apt/lists/*

# Install additional packages needed for ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-distutils \
    libboost-all-dev \
    libboost-dev \
&& rm -rf /var/lib/apt/lists/*

# sklearn dependencies
RUN apt-get update && apt-get install -y \
    gfortran \
    libatlas-base-dev \
    python3-scipy \
&& rm -rf /var/lib/apt/lists/*

# sklearn Python dependencies
RUN python3 -m pip install -U \
    Cython \
    wheel

# Install sklearn
RUN python3 -m pip install -U \
    scikit-learn

# Download and build nanosaur_isaac_ros
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
# Copy wstool isaac_ros.rosinstall
COPY isaac_ros.rosinstall isaac_ros.rosinstall

# Initialize ROS2 workspace
RUN pip3 install wheel && \
    pip3 install -U wstool && \
    wstool init $ROS_WS/src && \
    wstool merge -t $ROS_WS/src isaac_ros.rosinstall && \
    wstool update -t $ROS_WS/src

ADD libs/vpi1/ /usr/share/vpi1/
ADD libs/nvidia/vpi1/ /opt/nvidia/vpi1/

# Load ROS2 sources
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd /$ROS_WS && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

RUN rm -R /usr/share/vpi1/
RUN rm -R /opt/nvidia/vpi1/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Change workdir
WORKDIR $ROS_WS
# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

