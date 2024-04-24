""" Builds zed_interfaces. """

load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "ros2_interface_library",
    "cpp_ros2_interface_library",
)
load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_library",
)

ros2_interface_library(
    name = "zed_interfaces",
    srcs = glob([
        "msg/*.msg",
        "srv/*.srv",
    ]),
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:shape_msgs",
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:std_msgs",
    ],
)

cpp_ros2_interface_library(
    name = "_cpp_zed_interfaces",
    visibility = ["//visibility:private"],
    deps = [":zed_interfaces"],

)

ros2_cpp_library(
    name = "cpp_zed_interfaces",
    hdrs = glob([
        "zed_interfaces/**/*.hpp",
    ]),
    includes = [".",],
    visibility = ["//visibility:public"],
    deps = [":_cpp_zed_interfaces"],
    copts = ["-std=c++17"],
)
