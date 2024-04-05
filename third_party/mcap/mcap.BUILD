""" Builds mcap.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "mcap",
    hdrs = glob([
        "cpp/mcap/include/**/*.hpp",
        "cpp/mcap/include/**/*.inl",
    ]),
    includes = ["cpp/mcap/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@zstd",
        "@lz4//:lz4_frame",
    ],
)
