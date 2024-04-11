"""Loads the zstd library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "zstd",
        build_file = "//third_party/zstd:zstd.BUILD.bazel",
        sha256 = "8c29e06cf42aacc1eafc4077ae2ec6c6fcb96a626157e0593d5e82a34fd403c1",
        strip_prefix = "zstd-1.5.6",
        urls = ["https://github.com/facebook/zstd/releases/download/v1.5.6/zstd-1.5.6.tar.gz"],
    )
