load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "header",
    hdrs = glob(["**"]),
    includes = ["."],
)

cc_library(
    name = "core",
    includes = ["."],
    linkopts = [
        "-L/usr/local/lib",
        "-lopencv_core",
    ],
)

cc_library(
    name = "highgui",
    includes = ["."],
    linkopts = [
        "-L/usr/local/lib",
        "-lopencv_highgui",
    ],
    deps = [
        ":core",
        ":imgproc",
    ],
)

cc_library(
    name = "imgproc",
    includes = ["."],
    linkopts = [
        "-L/usr/local/lib",
        "-lopencv_imgproc",
    ],
    deps = [
        ":core",
    ],
)

cc_library(
    name = "imgcodecs",
    includes = ["."],
    linkopts = [
        "-L/usr/local/lib",
        "-lopencv_imgcodecs",
    ],
    deps = [
        ":core",
        ":imgproc",
    ],
)

cc_library(
    name = "calib3d",
    includes = ["."],
    linkopts = [
        "-L/usr/local/lib",
        "-lopencv_calib3d",
    ],
    deps = [
        ":core",
    ],
)