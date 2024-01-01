"""Loads the yaml-cpp library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "yaml-cpp",
        # Here you can use e.g. sha256sum cli utility to compute the sha sum.
        sha256 = "fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16",
        strip_prefix = "yaml-cpp-0.8.0",
        url = "https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz",
    )
