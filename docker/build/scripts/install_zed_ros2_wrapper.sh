#!/bin/bash

mkdir -p /workspace/src/
cd /workspace/src

# Check if the directory zed-ros2-wrapper already exists
# if [ ! -d "zed-ros2-wrapper" ]; then
#     git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
# else
#     echo "zed-ros2-wrapper directory already exists."
# fi

# cd ..

# sudo apt update \

# source /opt/ros/${ROS_DISTRO}/setup.bash \&& sudo apt-get update \
    
# rosdep update \

# rosdep install --from-paths src --ignore-src -r -y \

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
#         --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja --parallel-workers $(nproc) \

echo source $(pwd)/install/local_setup.bash >> /home/admin/.bashrc