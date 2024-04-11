load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "zed",
    srcs = [
        "lib/libsl_zed.so",
    ],
    hdrs = glob(["include/**/*"]),
    includes = ["include"],
    deps = [
        "@cuda//:compat",
        "@cuda//:headers"
    ],
    linkopts = [
        #"-L/usr/local/lib",
        "-lnvcuvid",
        "-lnvidia-encode"
    ],
)


cc_library(
    name = "zed_aarch64",
    srcs = [
        "lib/libsl_zed.so",
    ],
    hdrs = glob(["include/**/*"]),
    includes = ["include"],
    deps = [
        "@cuda//:compat_arm",
        "@cuda//:headers_arm"
    ]
)
