find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)  # Add this line

# Create the IO library
add_library(actuators SHARED
    MobileBaseActuator.cpp
)

# Include directories and link libraries
ament_target_dependencies(actuators
    rclcpp
    geometry_msgs
)
target_include_directories(actuators PUBLIC
        $<BUILD_INTERFACE:${TAREEQ_PROJECT_ROOT}> # Use the project root
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(actuators
        fmt::fmt  # This uses the imported target from the fmt package
)
