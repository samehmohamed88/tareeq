load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tensorrt8",
    srcs = [
        "targets/x86_64-linux-gnu/lib/libnvinfer.so",
        "targets/x86_64-linux-gnu/lib/libnvinfer.so.8",
        "targets/x86_64-linux-gnu/lib/libnvinfer.so.8.6.1",
        "targets/x86_64-linux-gnu/lib/libnvonnxparser.so",
        "targets/x86_64-linux-gnu/lib/libnvonnxparser.so.8",
        "targets/x86_64-linux-gnu/lib/libnvonnxparser.so.8.6.1",
        "targets/x86_64-linux-gnu/lib/libnvparsers.so",
        "targets/x86_64-linux-gnu/lib/libnvparsers.so.8",
        "targets/x86_64-linux-gnu/lib/libnvparsers.so.8.6.1",
    ],
    hdrs = glob(["include/**"]),
    includes = ["include"],
    deps = [
        "@cuda//:cublas",
        "@cuda//:cudart",
        "@cudnn8",
    ]
)

