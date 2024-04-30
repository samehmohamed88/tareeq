
# Create the IO library
add_library(SLAMClient SHARED
        SLAMServiceClient.cpp
        SLAMActionClient.cpp
)

ament_target_dependencies(SLAMClient
        rclcpp
        rclcpp_action
        isaac_ros_visual_slam_interfaces)

target_include_directories(SLAMClient PUBLIC
        $<BUILD_INTERFACE:${TAREEQ_PROJECT_ROOT}> # Use the project root
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS SLAMClient
        DESTINATION lib/${PROJECT_NAME})
