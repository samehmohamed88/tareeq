""" Builds zed_ros2_wrapper. """

load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_binary",
    "ros2_cpp_library",
)

ros2_cpp_library(
    name = "zed_ros2_wrapper",
    srcs = glob([
        "**/*.cpp",
       # ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_tools.cpp
       # ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_win_avg.cpp
       # ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/src/zed_camera_component.cpp
    ]),
    hdrs = glob([
        "**/*.hpp",
        #"include/robot_state_publisher/*.hpp"
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_tools.hpp
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_win_avg.hpp
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/sl_logging.hpp
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/include/visibility_control.hpp
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/sl_types.hpp
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_camera/include/zed_camera_component.hpp
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@orocos_kdl",
        "@ros2_rcutils//:rcutils",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_common_interfaces//:cpp_nav_msgs",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_geometry2//:tf2",
        "@ros2_geometry2//:tf2_ros",
        "@ros2_geometry2//:cpp_tf2_geometry_msgs",
        "@ros2_kdl_parser//:kdl_parser",
        "@ros2_rcl_interfaces//:cpp_builtin_interfaces",
        "@ros2_rcl_interfaces//:cpp_rcl_interfaces",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
        "@ros2_urdf//:urdf",
    ],
)
