load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cnpy",
    hdrs = ["cnpy.h"],
    includes = ["."],
    linkopts = [
        "-L/usr/local/lib",
        "-lcnpy",
    ],
)
