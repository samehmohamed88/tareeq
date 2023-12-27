"""Loads the rules_ros2 library"""
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_github_mvukov_rules_ros2",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "5ba2b2e9329e3da168739b10e2a094a3e6f7806aa75118a15a6956217e6f265b",
        strip_prefix = "rules_ros2-900fd0c5a0250b89cf429e2f960dd6dbce98a108",
        url = "https://github.com/mvukov/rules_ros2/archive/900fd0c5a0250b89cf429e2f960dd6dbce98a108.tar.gz",
    )
    
