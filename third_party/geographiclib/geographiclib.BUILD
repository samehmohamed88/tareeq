""" Builds geographiclib angles. """

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "geographiclib",
    linkopts = [
        "-L/usr/local/lib",
        "-lGeographicLib",
    ],
    hdrs = glob(["*"]),
    includes = ["."],
)
