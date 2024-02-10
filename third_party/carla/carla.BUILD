load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "carla",
    hdrs = glob([
        "include/**"
    ]),
    includes = [
        "include",
        "include/system",
    ],
    linkopts = [
        "-L/usr/local/libcarla-0.9.15/lib",
        #"-Wl",
        #"-Bstatic",
        "-lcarla_client",
        "-lrpc",
        "-lboost_filesystem",
        "-Bdynamic",
        "-lpng",
        #"-ltiff",
        #"-ljpeg",
        "-lRecast",
        "-lDetour",
        "-lDetourCrowd",
    ],
)
