load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "zed",
    srcs = [
        "lib/libsl_zed.so",
    ],
    hdrs = glob(["include/**/*"]),
    includes = ["include"],
)
