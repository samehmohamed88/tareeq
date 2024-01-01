"""Loads the zed_ros2_interfaces library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "zed_ros2_wrapper",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "0feb5c2a67c21ad4df1ed720d4bddfc5e763084dd16b66c9761fdf4a1d64f624",
        build_file = "//third_party/zed_ros2_wrapper:zed.BUILD",
        strip_prefix = "foxy-humble-v4.0.5",
        url = "https://github.com/stereolabs/zed-ros2-wrapper/archive/refs/tags/foxy-humble-v4.0.5.tar.gz",
    )
