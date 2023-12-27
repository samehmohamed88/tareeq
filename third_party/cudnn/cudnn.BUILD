load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cudnn8",
    srcs = glob([
        "lib/libcudnn.so*",
        "lib/libcudnn*_infer.so*",
        "lib/libcudnn*_train.so*",
    ]),
    hdrs = glob(["include/**"]),
    includes = ["include"]
)

filegroup(
    name = "cudnn_files",
    srcs = glob([
        "lib/libcudnn.so*",
        "lib/libcudnn*_infer.so*",
        "lib/libcudnn*_train.so*",
    ]),
)
