""" Builds geographic_info. """

load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "ros2_interface_library",
    "cpp_ros2_interface_library"
)

ros2_interface_library(
    name = "geographic_msgs",
    srcs = glob([
        "geographic_msgs/msg/*.msg",
        "geographic_msgs/srv/*.srv",
    ]),
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:geometry_msgs",
        "@ros2_common_interfaces//:std_msgs",
        "@ros2_unique_identifier_msgs//:unique_identifier_msgs",
    ],
)

cpp_ros2_interface_library(
    name = "cpp_geographic_msgs",
    visibility = ["//visibility:public"],
    deps = [":geographic_msgs"],
)