"""Loads the zed_ros2_interfaces library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "zed_ros2_wrapper",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "defd1f27b34a7989eeef5b31841938b0b836b4c25aeb8a15facf2d5b31ba8c8b",
        build_file = "//third_party/ros2/zed_ros2/zed_ros2_wrapper:zed_ros2_wrapper.BUILD",
        strip_prefix = "zed-ros2-wrapper-humble-v4.0.8",
        url = "https://github.com/stereolabs/zed-ros2-wrapper/archive/refs/tags/humble-v4.0.8.tar.gz",
    )
