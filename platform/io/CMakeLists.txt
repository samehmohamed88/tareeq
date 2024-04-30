
# Create the IO library
add_library(${IO_LIBRARY_NAME} SHARED
  BoostSerialDevice.cpp
  BoostSerialDeviceManager.cpp
  #IOInterface.cpp
  #Status.cpp
)

# Include directories and link libraries
ament_target_dependencies(${IO_LIBRARY_NAME} rclcpp)

target_include_directories(${IO_LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${TAREEQ_PROJECT_ROOT}> # Use the project root
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${IO_LIBRARY_NAME}
        fmt::fmt  # This uses the imported target from the fmt package
)