#!/bin/bash
set -xe

apt-get update && apt-get install -y \
    sudo \
    udev \
    software-properties-common \
    apt-transport-https \
    bash-completion \
    build-essential \
    ca-certificates \
    clang-format \
    cmake \
    curl \
    git \
    gnupg2 \
    iputils-ping \
    locales \
    lsb-release \
    rsync \
    wget \
    vim \
    unzip \
    mlocate \
    libgoogle-glog-dev \
    libeigen3-dev \
    libasio-dev \
    libbullet-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    python3-opencv \
    v4l-utils \
    mesa-utils \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libv4l-dev \
    kmod \
    nvv4l2 \
    libgeographic-dev \
    ros-humble-geographic-info \
    ros-humble-nmea-msgs \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-eigen3-cmake-module
