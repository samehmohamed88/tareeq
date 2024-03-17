"""Loads the rules_ros2 library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_github_mvukov_rules_ros2",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "2d03d299a2d8bd9ea21a9788eddf31649440d87a98b85e4f1bd1d6d5d9d55969",
        strip_prefix = "rules_ros2-a3d796e544a86fcd00cb86184ac0487335aa51e7",
        url = "https://github.com/samehmohamed88/rules_ros2/archive/a3d796e544a86fcd00cb86184ac0487335aa51e7.tar.gz",
    )
