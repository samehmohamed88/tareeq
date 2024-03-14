"""Loads the ros2_angles library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "ros2_angles",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "25f5294d688e2e21583aa5e7fea64476f6a4a88ba59030b695f9d33682874efd",
        build_file = "//third_party/ros2_angles:ros2_angles.BUILD",
        strip_prefix = "angles-1.16.0",
        url = "https://github.com/ros/angles/archive/refs/tags/1.16.0.tar.gz",
    )
