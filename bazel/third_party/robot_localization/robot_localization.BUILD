""" Builds robot_localization. """

load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "ros2_interface_library",
    "cpp_ros2_interface_library",
)

load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_binary",
    "ros2_cpp_library",
)

ros2_interface_library(
    name = "robot_localization",
    srcs = glob([
        "srv/*.srv",
    ]),
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:shape_msgs",
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:diagnostic_msgs",
        "@geographic_info//:geographic_msgs",
    ],
)

cpp_ros2_interface_library(
    name = "_cpp_robot_localization",
    visibility = ["//visibility:public"],
    deps = [":robot_localization"],
)

ros2_cpp_library(
    name = "cpp_robot_localization",
    hdrs = glob([
        "**/*",
    ]),
    includes = [".",],
    visibility = ["//visibility:public"],
    deps = [":_cpp_robot_localization"],
)

ros2_cpp_library(
    name = "robot_localization_package",
    srcs = glob([
        "src/*.cpp",
    ]),
    hdrs = glob([
        "include/**/*.hpp",
    ]),
    includes = [
        "include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":cpp_robot_localization",
        "@boost//:accumulators",
        "@ros2_angles",
        "@ros2_common_interfaces//:cpp_diagnostic_msgs",
        "@ros2_diagnostics//:cpp_diagnostic_updater",
        "@geographic_info//:cpp_geographic_msgs",
        "@geographiclib",
        "@ros2_common_interfaces//:cpp_geometry_msgs",
        "@ros2_message_filters//:message_filters",
        "@ros2_common_interfaces//:cpp_nav_msgs",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_common_interfaces//:cpp_std_srvs",
        "@ros2_geometry2//:tf2",
        "@ros2_geometry2//:cpp_tf2_msgs",
        "@ros2_geometry2//:tf2_eigen",
        "@ros2_geometry2//:cpp_tf2_geometry_msgs",
        "@ros2_geometry2//:tf2_ros",
        "@yaml-cpp",
    ],
)
