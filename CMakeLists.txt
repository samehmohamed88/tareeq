cmake_minimum_required(VERSION 3.5)
project(tareeq)

# Global configurations
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add modules directory
add_subdirectory(av)
add_subdirectory(platform)

# Install launch files, etc.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Specific ROS2 package configuration
ament_package()

ament_package()
