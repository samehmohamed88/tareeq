cmake_minimum_required(VERSION 3.5)
project(tareeq)

# Define a variable to hold the root path of the project
set(TAREEQ_PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(TAREEQ_AV TareeqAV)
set(IO_LIBRARY_NAME tareeq_platform_io)
set(ACTUATOR_LIBRARY_NAME tareeq_actuators)

# Global configurations
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # Assuming you are using rclcpp for ROS2
find_package(rclcpp_action REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_package(fmt)


# Add modules
add_subdirectory(platform)
add_subdirectory(system)
add_subdirectory(samples)


# Install launch files, etc.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Specific ROS2 package configuration
ament_package()
