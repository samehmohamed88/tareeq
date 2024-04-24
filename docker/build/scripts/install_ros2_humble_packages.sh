#!/bin/bash
set -xe

apt-get update && apt-get install -y \
    ros-humble-geographic-info \
    ros-humble-nmea-msgs \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-eigen3-cmake-module \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-stereo-image-proc \
    ros-humble-foxglove-bridge \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup
