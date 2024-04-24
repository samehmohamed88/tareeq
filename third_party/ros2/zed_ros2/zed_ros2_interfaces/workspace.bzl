"""Loads the zed_ros2_interfaces library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "zed_interfaces",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "763193350dfcc43e1b11324efe7912958bfa12b40b3c34222d9c530febd81c03",
        build_file = "//third_party/zed_ros2_interfaces:zed.BUILD",
        strip_prefix = "zed-ros2-interfaces-foxy-humble-v4.0.5",
        url = "https://github.com/stereolabs/zed-ros2-interfaces/archive/refs/tags/foxy-humble-v4.0.5.tar.gz",
    )
