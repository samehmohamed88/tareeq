load("@rules_cuda//cuda:defs.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "yololayer",
    srcs = ["yololayer.cu"],
    hdrs = [
        "cuda_utils.h",
        "macros.h",
        "yololayer.h",
    ],
    deps = [
        "@cuda//:cublas",
        "@cuda//:cudart",
        "@tensorrt8",
    ],
)

cc_binary(
    name = "yolop",
    srcs = [
        "common.hpp",
        "logging.h",
        "macros.h",
        "utils.h",
        "yolop.cpp",
        "yolop.hpp",
    ],
    deps = [
        ":yololayer",
        "@opencv//:header",
        "@opencv//:imgcodecs",
    ],
)
