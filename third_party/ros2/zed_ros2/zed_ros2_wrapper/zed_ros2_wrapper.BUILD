""" Builds zed_ros2_wrapper. """

load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_binary",
    "ros2_cpp_library",
)

ros2_cpp_library(
    name = "zed_components",
    srcs = glob(
        include = ["zed_components/**/*.cpp"],
        exclude = ["zed_components/**/cost_traversability.cpp"],
    ),
    hdrs = glob(
        include = ["zed_components/**/*.hpp"],
        exclude = ["zed_components/**/cost_traversability.hpp"],
   ),
    includes = [
        "zed_components/src/include",
        "zed_components/src/zed_camera/include",
        "zed_components/src/tools/include",
    ],
    visibility = ["//visibility:public"],
    copts = [
            "-DFOUND_HUMBLE=1",  # Define FOUND_HUMBLE
            # Add other necessary compiler options here
    ],
    deps = [
        "@zed",
        "@zed_interfaces//:cpp_zed_interfaces",
        "@ros2_rclcpp//:rclcpp_components",
        "@ros2_geometry2//:tf2",
        "@ros2_geometry2//:tf2_ros",
        "@ros2_geometry2//:cpp_tf2_geometry_msgs",
        "@robot_localization//:robot_localization_package",
        "@ros2_diagnostics//:cpp_diagnostic_updater",
        "@ros2_image_common//:image_transport",
        "@geographic_info//:cpp_geographic_msgs",
        "@ros2_common_interfaces//:cpp_diagnostic_msgs",
        "@ros2_common_interfaces//:cpp_std_srvs",
        "@ros2_common_interfaces//:cpp_stereo_msgs",
        "@ros2_common_interfaces//:cpp_visualization_msgs",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_common_interfaces//:cpp_nav_msgs",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
        "@ros2_common_interfaces//:cpp_std_msgs",
      ]
)


ros2_cpp_library(
    name = "zed_ros2_wrapper",
    srcs = glob([
        "**/*.cpp",
    ]),
    hdrs = glob([
        "**/*.hpp",
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
