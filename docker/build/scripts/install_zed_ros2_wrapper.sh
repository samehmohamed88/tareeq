#!/bin/bash

sudo chown -R admin:admin /usr/local/zed /workspace \
    && mkdir -p /workspace/src/ \
    && cd /workspace/src \
    && git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    #  \
    # && cd .. \
    # && sudo apt update \
    # && source /opt/ros/${ROS_DISTRO}/setup.bash \
    # && sudo apt-get update \
    # && rosdep update \
    # && rosdep install --from-paths src --ignore-src -r -y \
    # && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    #     --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja --parallel-workers $(nproc) \
    # && echo source $(pwd)/install/local_setup.bash >> /home/admin/.bashrc
