load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sfml",
    srcs = glob(["lib/*.so*"]),
    hdrs = glob(["include/**/*"]),
    includes = ["include"],

)
