#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# sudo apt-get update
# rosdep update

# Restart udev daemon
sudo service udev restart

sudo chown -R admin:admin ~/.local ~/.java ~/.cache ~/.config

sudo chown -R admin:admin /usr/local/zed /workspace

mkdir -p /workspace/maps

sudo cp /workspace/src/tareeq/config/zed.yaml /opt/ros/humble/share/isaac_ros_visual_slam/config/zed.yaml

/usr/local/bin/scripts/install_zed_ros2_wrapper.sh

colcon build --event-handlers console_cohesion+ \
  --packages-select tareeq --symlink-install --parallel-workers `nproc`

source /workspace/install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml &

ros2 launch tareeq tareeqav_launch.py

#cd /workspace/src/tareeq \
#    && curl -fsSL https://raw.githubusercontent.com/bazelbuild/bazel/$(cat .bazelversion)/scripts/bazel-complete-header.bash >> ~/.bashrc \
#    && curl -fsSL https://raw.githubusercontent.com/bazelbuild/bazel/$(cat .bazelversion)/scripts/bazel-complete-template.bash >> ~/.bashrc \
#    && bazel help completion >> ~/.bashrc

$@
