"""Loads the robot_localization library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "robot_localization",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "17a9bd305baeebe327cf92f3d6b17cf714620a480a36a158bb04354e3aba5b9b",
        build_file = "//third_party/robot_localization:robot_localization.BUILD",
        strip_prefix = "robot_localization-3.5.1",
        url = "https://github.com/cra-ros-pkg/robot_localization/archive/refs/tags/3.5.1.tar.gz",
    )
