"""Loads the lz4 library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "lz4",
        build_file = "//third_party/lz4:lz4.BUILD.bazel",
        sha256 = "4ec935d99aa4950eadfefbd49c9fad863185ac24c32001162c44a683ef61b580",
        strip_prefix = "lz4-1.9.3",
        urls = ["https://github.com/lz4/lz4/archive/refs/tags/v1.9.3.zip"],
    )
