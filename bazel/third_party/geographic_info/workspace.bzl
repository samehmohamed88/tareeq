"""Loads the zed_ros2_interfaces library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "geographic_info",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "adb6582b4ba32126da17144d7ab6689873d26b0f854eee2125b48895b0df957a",
        build_file = "//third_party/geographic_info:geographic_info.BUILD",
        strip_prefix = "geographic_info-1.0.5",
        url = "https://github.com/ros-geographic-info/geographic_info/archive/refs/tags/1.0.5.tar.gz",
    )
