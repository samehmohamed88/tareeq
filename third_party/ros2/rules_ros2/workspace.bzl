"""Loads the rules_ros2 library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_github_mvukov_rules_ros2",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "93bab5e6e167e51181dd0631ef70fd9f5da75208662074becf11ccb21e5ab214",
        strip_prefix = "rules_ros2-bd6192d10517eb0cc5056a3b7ed4d88e1af1588b",
        url = "https://github.com/samehmohamed88/rules_ros2/archive/bd6192d10517eb0cc5056a3b7ed4d88e1af1588b.tar.gz",
    )
