import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # os.environ['RCUTILS_LOGGING_SEVERITY'] = 'DEBUG'

    # Path to the external launch file (e.g., from the SLAM package)
    nvidia_vslam_launch_file = os.path.join(
        get_package_share_directory('tareeq'),  # Update with actual package name
        'launch',
        'isaac_ros_visual_slam_zed.launch.py'  # Update with actual launch file name
    )

    # Your custom node for motor actuation
    motor_actuation_node = Node(
        package='tareeq',
        executable='TareeqAV',
        name='tareeqav',
        output='screen',
        # arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nvidia_vslam_launch_file)
        ),
        TimerAction(
            period=5.0,  # Delay to ensure SLAM components are initialized
            actions=[motor_actuation_node]
        )
    ])
