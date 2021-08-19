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


