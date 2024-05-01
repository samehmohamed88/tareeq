import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    shared_container_name = "tareeqav_container"

    bringup_dir = get_package_share_directory('tareeq')

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')

    global_frame = LaunchConfiguration('global_frame',
                                       default='odom')

    # Create a shared container to hold composable nodes
    # for speedups through intra process communication.
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen'
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'zed2.launch.py')]),
        launch_arguments={
            'component_container_name': shared_container_name}.items())

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nvblox_examples_bringup'), 'launch', 'nvblox', 'nvblox.launch.py')]),
        launch_arguments={'global_frame': global_frame,
                          'setup_for_zed': 'True',
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items())

    return LaunchDescription([
        run_rviz_arg,
        shared_container,
        zed_launch,
        nvblox_launch,
        ])