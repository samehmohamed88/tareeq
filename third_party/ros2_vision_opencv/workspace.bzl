"""Loads the ros2_angles library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "ros2_vision_opencv",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "53ff29fbe8da48333bc51dba1125bd3b5aef346c8353cf47b2628b4daf664b97",
        build_file = "//third_party/ros2_vision_opencv:ros2_vision_opencv.BUILD",
        strip_prefix = "vision_opencv-3.1.3",
        url = "https://github.com/ros-perception/vision_opencv/archive/refs/tags/3.1.3.tar.gz",
    )
