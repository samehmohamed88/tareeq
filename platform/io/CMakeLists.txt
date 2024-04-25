# Create the IO library
add_library(io SHARED
  BoostSerialDevice.cpp
  BoostSerialDeviceManager.cpp
  IOInterface.cpp
  Status.cpp
)

# Include directories and link libraries
ament_target_dependencies(io rclcpp)
target_include_directories(io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # Use the current source directory
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>   # Maintain standard ROS2 install structure
)
