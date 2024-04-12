"""Loads the mcap library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "mcap",
        build_file = "//third_party/mcap:mcap.BUILD",
        sha256 = "41acf6e85d75556c64407f077e05492d31db1f099e07242ef04364bb2939acf1",
        strip_prefix = "mcap-releases-cpp-v1.3.0",
        url = "https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v1.3.0.tar.gz",
    )
