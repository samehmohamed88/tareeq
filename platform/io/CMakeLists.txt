# Create the IO library
add_library(io SHARED
  BoostSerialDevice.cpp
  BoostSerialDeviceManager.cpp
  #IOInterface.cpp
  #Status.cpp
)

# Include directories and link libraries
ament_target_dependencies(io rclcpp)
target_include_directories(io PUBLIC
  $<BUILD_INTERFACE:${TAREEQ_PROJECT_ROOT}> # Use the project root
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(io
  fmt::fmt  # This uses the imported target from the fmt package
)