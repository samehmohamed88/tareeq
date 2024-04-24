""" Builds ros2 vision_opencv. """

load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_binary",
    "ros2_cpp_library",
)

package(default_visibility = ["//visibility:public"])

ros2_cpp_library(
    name = "cv_bridge",
    srcs = glob(["cv_bridge/src/*.cpp"]),
    hdrs = glob(["cv_bridge/include/cv_bridge/*.hpp"]),
    includes = ["cv_bridge/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@opencv//:core",
        "@opencv//:imgproc",
        "@opencv//:imgcodecs",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rcpputils//:rcpputils",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
    ],
)