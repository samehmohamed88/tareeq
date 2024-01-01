"""Loads the rules_ros2 library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_github_mvukov_rules_ros2",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "95f4178da63e0c629379d679aaf9f77323f114ca192a8f601c6d53a6114437af",
        strip_prefix = "rules_ros2-1fcde3efdcf22a9aa50bf9454f7bc0f93fbf44b6",
        url = "https://github.com/samehmohamed88/rules_ros2/archive/1fcde3efdcf22a9aa50bf9454f7bc0f93fbf44b6.tar.gz",
    )
