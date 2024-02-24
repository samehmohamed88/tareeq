package(default_visibility = ["//visibility:public"])

cc_library(
    name = "Component",
    srcs = [
        "Component.h",
    ],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
    ],
)

cc_library(
    name = "ROSComponentFactory",
    srcs = ["ROSComponentFactory.cc"],
    hdrs = ["ROSComponentFactory.h"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
    ],
)

cc_library(
    name = "StopWatch",
    srcs = ["StopWatch.cc"],
    hdrs = ["StopWatch.h"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
    ],
)
