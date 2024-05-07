find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)  # Add this line

# Create the IO library
add_library(${ACTUATOR_LIBRARY_NAME} SHARED
        MobileBaseActuator.cpp
        SabertoothMotorController.cpp
)

# Include directories and link libraries
ament_target_dependencies(${ACTUATOR_LIBRARY_NAME}
    rclcpp
    geometry_msgs
)
target_include_directories(${ACTUATOR_LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${TAREEQ_PROJECT_ROOT}> # Use the project root
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${ACTUATOR_LIBRARY_NAME}
        ${IO_LIBRARY_NAME}
        fmt::fmt  # This uses the imported target from the fmt package
)
