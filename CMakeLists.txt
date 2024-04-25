cmake_minimum_required(VERSION 3.5)
project(tareeq)

# Define a variable to hold the root path of the project
set(TAREEQ_PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Global configurations
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # Assuming you are using rclcpp for ROS2
find_package(fmt)


# Add modules directory
#add_subdirectory(av)
add_subdirectory(platform)

# Install launch files, etc.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Specific ROS2 package configuration
ament_package()
