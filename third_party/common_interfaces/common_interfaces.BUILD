""" Builds robot_state_publisher. """

load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "ros2_interface_library",
    "cpp_ros2_interface_library",
    "ros2_cpp_library"
)

ros2_interface_library(
    name = "stereo_msgs",
    srcs = glob([
        "sensor_msgs/msg/*.msg",
        "sensor_msgs/srv/*.srv",
    ]),
    visibility = ["//visibility:public"],
    deps = [
        ":geometry_msgs",
        ":std_msgs",
    ],
)

cpp_ros2_interface_library(
    name = "_cpp_stereo_msgs",
    visibility = ["//visibility:private"],
    deps = [":stereo_msgs"],
)

ros2_cpp_library(
    name = "cpp_stereo_msgs",
    hdrs = glob([
        "sensor_msgs/include/**/*.hpp",
    ]),
    includes = ["sensor_msgs/include"],
    visibility = ["//visibility:public"],
    deps = [":_cpp_stereo_msgs"],
)