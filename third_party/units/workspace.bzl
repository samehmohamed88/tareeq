""" module to load units library """

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "units",
        build_file = "//third_party/units:units.BUILD",
        sha256 = "cfed3cc3bf455a083a1840b7da1b6f65460c13987f5748e3dc4d315a404e29a2",
        strip_prefix = "units-2.3.3",
        urls = [
            "https://github.com/nholthaus/units/archive/refs/tags/v2.3.3.zip",
        ],
    )
