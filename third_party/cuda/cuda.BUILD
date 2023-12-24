load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

exports_files([
    "bin/nvcc",
])

cc_library(
    name = "headers",
    hdrs = glob(["targets/x86_64-linux/include/**"]),
    includes = ["targets/x86_64-linux/include"],
)

cc_library(
    name = "cudart",
    srcs = [
        "targets/x86_64-linux/lib/libcudart.so",
        "targets/x86_64-linux/lib/libcudart.so.12",
        "targets/x86_64-linux/lib/libcudart.so.12.1.105",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "cublas",
    srcs = [
        "targets/x86_64-linux/lib/libcublas.so",
        "targets/x86_64-linux/lib/libcublas.so.12",
        "targets/x86_64-linux/lib/libcublas.so.12.1.3.1",
        "targets/x86_64-linux/lib/libcublasLt.so",
        "targets/x86_64-linux/lib/libcublasLt.so.12",
        "targets/x86_64-linux/lib/libcublasLt.so.12.1.3.1",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "cufft",
    srcs = [
        "targets/x86_64-linux/lib/libcufft.so",
        "targets/x86_64-linux/lib/libcufft.so.11",
        "targets/x86_64-linux/lib/libcufft.so.11.0.2.54",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "cuinj64",
    srcs = [
        "targets/x86_64-linux/lib/libcuinj64.so",
        "targets/x86_64-linux/lib/libcuinj64.so.12.1",
        "targets/x86_64-linux/lib/libcuinj64.so.12.1.105",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "cupti",
    srcs = [
        "extras/CUPTI/lib64/libcupti.so",
        "extras/CUPTI/lib64/libcupti.so.12",
        "extras/CUPTI/lib64/libcupti.so.2023.1.1",
    ],
    hdrs = glob(["extras/CUPTI/include/**"]),
    includes = ["extras/CUPTI/include"],
    deps = [
        "headers",
        "@av_toolchain//:util",
    ],
)

cc_library(
    name = "curand",
    srcs = [
        "targets/x86_64-linux/lib/libcurand.so",
        "targets/x86_64-linux/lib/libcurand.so.10",
        "targets/x86_64-linux/lib/libcurand.so.10.3.2.106",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "cusolver",
    srcs = [
        "targets/x86_64-linux/lib/libcusolver.so",
        "targets/x86_64-linux/lib/libcusolver.so.11",
        "targets/x86_64-linux/lib/libcusolver.so.11.4.5.107",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "cusparse",
    srcs = [
        "targets/x86_64-linux/lib/libcusparse.so",
        "targets/x86_64-linux/lib/libcusparse.so.12",
        "targets/x86_64-linux/lib/libcusparse.so.12.1.0.106",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "nvToolsExt",
    srcs = [
        "targets/x86_64-linux/lib/libnvToolsExt.so",
        "targets/x86_64-linux/lib/libnvToolsExt.so.1",
        "targets/x86_64-linux/lib/libnvToolsExt.so.1.0.0",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "nvrtc",
    srcs = [
        "targets/x86_64-linux/lib/libnvrtc.so",
        "targets/x86_64-linux/lib/libnvrtc.so.12",
        "targets/x86_64-linux/lib/libnvrtc.so.12.1.105",
    ],
    deps = [
        "headers",
    ],
)

cc_library(
    name = "npp",
    srcs = [
        "targets/x86_64-linux/lib/libnppc.so",
        "targets/x86_64-linux/lib/libnppc.so.12",
        "targets/x86_64-linux/lib/libnppc.so.12.1.0.40",
        "targets/x86_64-linux/lib/libnppig.so",
        "targets/x86_64-linux/lib/libnppig.so.12",
        "targets/x86_64-linux/lib/libnppig.so.12.1.0.40",
    ],
    deps = [
        ":headers",
    ],
)

filegroup(
    name = "cuda_files",
    srcs = glob([
        "targets/x86_64-linux/lib/libcudart.so*",
        "targets/x86_64-linux/lib/libcublas.so*",
        "targets/x86_64-linux/lib/libcublasLt.so*",
        "targets/x86_64-linux/lib/libcufft.so*",
        "targets/x86_64-linux/lib/libcurand.so*",
        "targets/x86_64-linux/lib/libcusolver.so*",
        "targets/x86_64-linux/lib/libcusparse.so*",
        "targets/x86_64-linux/lib/libnvToolsExt.so*",
        "targets/x86_64-linux/lib/libnvrtc.so*",
        "targets/x86_64-linux/lib/libnppig.so*",
        "targets/x86_64-linux/lib/libnppc.so*",
        "extras/CUPTI/lib64/libcupti.so*",
    ]),
)

filegroup(
    name = "compiler_deps",
    srcs = glob([
        "bin/**",
        "targets/x86_64-linux/lib/**",
        "nvvm/**",
        "targets/x86_64-linux/include/**",
    ]),
)

filegroup(
    name = "cudart_files",
    srcs = glob([
        "targets/x86_64-linux/lib/libcudart.so*",
    ]),
)
