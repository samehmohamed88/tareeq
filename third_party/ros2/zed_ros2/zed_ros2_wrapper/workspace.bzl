"""Loads the zed_ros2_interfaces library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "zed_ros2_wrapper",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "d4d1b970c7e95bbf338547eaa53c373a737d05c4e89a8daa87572b58d69e2d1d",
        build_file = "//third_party/ros2/zed_ros2/zed_ros2_wrapper:zed_ros2_wrapper.BUILD",
        strip_prefix = "zed-ros2-interfaces-humble-v4.0.8",
        url = "https://github.com/stereolabs/zed-ros2-interfaces/archive/refs/tags/humble-v4.0.8.tar.gz",
    )
