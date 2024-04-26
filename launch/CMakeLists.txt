###############################################################################
# SOURCES
set(TAREEQ_LAUNCH_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/TareeqAV.cpp
)

###############################################################################
# Bin and Install
add_executable(tareeq_system
    ${TAREEQ_LAUNCH_SRC}
)

###############################################################################
# INCLUDES and LIBS
target_include_directories(tareeq_system PUBLIC
        $<BUILD_INTERFACE:${TAREEQ_PROJECT_ROOT}> # Use the project root
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(tareeq_system
    rclcpp
#    rclcpp_components
)

target_link_libraries(tareeq_system PUBLIC
        ${IO_LIBRARY_NAME}
)
