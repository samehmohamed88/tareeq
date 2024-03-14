"""Loads the common_interfaces library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        name = "common_interfaces",
        build_file = "//third_party/common_interfaces:common_interfaces.BUILD",
        sha256 = "f4be9343a4c028fcf5403d90c120bca78aea1bbe2a04ae9838a7f73c347366c6",
        strip_prefix = "common_interfaces-4.2.3",
        url = "https://github.com/ros2/common_interfaces/archive/refs/tags/4.2.3.tar.gz",
    )
