add_executable(${TAREEQ_AV}
        TareeqAV.cpp)

add_executable(MapServerClient
        MapServerClient.cpp)

ament_target_dependencies(${TAREEQ_AV}
        rclcpp
        std_msgs)

ament_target_dependencies(MapServerClient
        rclcpp
        rclcpp_action
        isaac_ros_visual_slam_interfaces)

target_link_libraries(${TAREEQ_AV}
        ${ACTUATOR_LIBRARY_NAME}
        SLAMClient
        ${IO_LIBRARY_NAME}
)

install(TARGETS ${TAREEQ_AV}
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS MapServerClient
        DESTINATION lib/${PROJECT_NAME})
