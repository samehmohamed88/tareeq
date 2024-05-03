add_executable(${TAREEQ_AV}
        TareeqAV.cpp)

add_executable(CmdVelocity
        CmdVelocity.cpp)

add_executable(MapServerClient
        MapServerClient.cpp)

ament_target_dependencies(${TAREEQ_AV}
        rclcpp
        std_msgs)

ament_target_dependencies(CmdVelocity
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

target_link_libraries(CmdVelocity
        ${ACTUATOR_LIBRARY_NAME}
        ${IO_LIBRARY_NAME}
)

install(TARGETS CmdVelocity
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS MapServerClient
        DESTINATION lib/${PROJECT_NAME})
