add_executable(${TAREEQ_AV}
        TareeqAV.cpp)

ament_target_dependencies(${TAREEQ_AV}
        rclcpp
        std_msgs)

target_link_libraries(${TAREEQ_AV}
        ${ACTUATOR_LIBRARY_NAME}
        SLAMClient
        ${IO_LIBRARY_NAME}
)

install(TARGETS ${TAREEQ_AV}
        DESTINATION lib/${PROJECT_NAME})
