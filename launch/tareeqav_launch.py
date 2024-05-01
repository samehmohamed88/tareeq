import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    os.environ['RCUTILS_LOGGING_SEVERITY'] = 'INFO'

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
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    )

    # Robot State Publisher node (publishing static tfs for the camera)
    rsp_node = Node(
        package='robot_state_publisher',
        namespace='zed2',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', 'zed2', ' ',
                    'camera_model:=', 'zed2', ' ',
                    'base_frame:=', 'base_link', ' ',
                ])
        }]
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'nav2.launch.py')),
        launch_arguments={'global_frame': global_frame}.items())


    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'zed2.launch.py')]),
        launch_arguments={
            'component_container_name': shared_container_name}.items())

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'vslam.launch.py')]),
        launch_arguments={'output_odom_frame_name': global_frame,
                          'setup_for_isaac_sim': 'False',
                          # Flatten VIO to 2D (assuming the robot only moves horizontally).
                          # This is needed to prevent vertical odometry drift.
                          'run_odometry_flattening': 'True',
                          'component_container_name': shared_container_name}.items())

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'nvblox.launch.py')]),
        launch_arguments={'global_frame': global_frame,
                          'setup_for_zed': 'True',
                          'component_container_name': shared_container_name}.items())

    return LaunchDescription([
        run_rviz_arg,
        shared_container,
        rsp_node,
        nav2_launch,
        zed_launch,
        vslam_launch,
        nvblox_launch,
        ])