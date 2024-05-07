# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes, Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (SetParameter, SetParametersFromFile, SetRemap)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    bringup_dir = get_package_share_directory('tareeq')
    base_config_dir = os.path.join(bringup_dir, 'config')

    zed_config = os.path.join(
        base_config_dir, 'nvblox_zed.yaml')

    setup_for_zed = IfCondition(
        LaunchConfiguration('setup_for_zed', default='True'))
    
    # attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    component_container_name_arg = LaunchConfiguration('component_container_name', default='tareeqav_container')
    
    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            ComposableNode(
            name='nvblox_node',
            package='nvblox_ros',
            plugin='nvblox::NvbloxNode')])

    group_action = GroupAction([

        # Set parameters with specializations
        SetParametersFromFile(zed_config,
                              condition=setup_for_zed),
        SetParameter(name='global_frame',
                     value=LaunchConfiguration('global_frame', default='odom')),

        # Remappings for zed
	    SetRemap(src=['depth/image'],
                 dst=['/zed2/zed_node/depth/depth_registered'],
                 condition=setup_for_zed),
        SetRemap(src=['depth/camera_info'],
                 dst=['/zed2/zed_node/depth/camera_info'],
                 condition=setup_for_zed),
        SetRemap(src=['color/image'],
                 dst=['/zed2/zed_node/rgb/image_rect_color'],
                 condition=setup_for_zed),
        SetRemap(src=['color/camera_info'],
                 dst=['/zed2/zed_node/rgb/camera_info'],
                 condition=setup_for_zed),
        SetRemap(src=['pose'],
                 dst=['/zed2/zed_node/pose'],
                 condition=setup_for_zed),

        # Include the node container
        load_composable_nodes
    ])

    return LaunchDescription([group_action])
