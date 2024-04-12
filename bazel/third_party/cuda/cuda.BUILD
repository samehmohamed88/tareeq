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
    name = "headers_arm",
    hdrs = glob(["targets/aarch64-linux/include/**"]),
    includes = ["targets/aarch64-linux/include"],
)

cc_library(
    name = "compat_arm",
    srcs = [
        "compat/libcuda.so",
        "compat/libcuda.so.1",
        "compat/libcuda.so.1.1",
        "compat/libnvidia-nvvm.so",
        "compat/libnvidia-nvvm.so.4",
        "compat/libnvidia-ptxjitcompiler.so.1",
    ],
    deps = [
        "headers_arm",
    ],
)

cc_library(
    name = "cudart_arm",
    srcs = [
        "targets/aarch64-linux/lib/libcudart.so",
        "targets/aarch64-linux/lib/libcudart.so.12",
        "targets/aarch64-linux/lib/libcudart.so.12.2.140",
    ],
    deps = [
        "headers_arm",
    ],
)

cc_library(
    name = "cublas_arm",
    srcs = [
        "targets/aarch64-linux/lib/libcublas.so",
        "targets/aarch64-linux/lib/libcublas.so.12",
        "targets/aarch64-linux/lib/libcublas.so.12.1.3.1",
        "targets/aarch64-linux/lib/libcublasLt.so",
        "targets/aarch64-linux/lib/libcublasLt.so.12",
        "targets/aarch64-linux/lib/libcublas.so.12.2.5.6",
    ],
    deps = [
        "headers_arm",
    ],
)

cc_library(
    name = "compat",
    srcs = [
        "compat/libcuda.so",
        "compat/libcuda.so.1",
        "compat/libcuda.so.530.30.02",
        "compat/libnvidia-nvvm.so",
        "compat/libnvidia-nvvm.so.4",
        "compat/libnvidia-nvvm.so.530.30.02",
        "compat/libnvidia-ptxjitcompiler.so.1",
        "compat/libnvidia-ptxjitcompiler.so.530.30.02",
    ],
    deps = [
        "headers",
    ],
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

#cc_library(
#    name = "cufft",
#    srcs = [
#        "targets/x86_64-linux/lib/libcufft.so",
#        "targets/x86_64-linux/lib/libcufft.so.11",
#        "targets/x86_64-linux/lib/libcufft.so.11.0.8.103",
#    ],
#    deps = [
#        "headers",
#    ],
#)

#cc_library(
#    name = "cuinj64",
#    srcs = [
#        "targets/x86_64-linux/lib/libcuinj64.so",
#        "targets/x86_64-linux/lib/libcuinj64.so.12.2",
#        "targets/x86_64-linux/lib/libcuinj64.so.12.2.142",
#    ],
#    deps = [
#        "headers",
#    ],
#)

#cc_library(
#    name = "cupti",
#    srcs = [
#        "extras/CUPTI/lib64/libcupti.so",
#        "extras/CUPTI/lib64/libcupti.so.12",
#        "extras/CUPTI/lib64/libcupti.so.2023.2.2",
#    ],
#    hdrs = glob(["extras/CUPTI/include/**"]),
#    includes = ["extras/CUPTI/include"],
#    deps = [
#        "headers",
#
#    ],
#)

#cc_library(
#    name = "curand",
#    srcs = [
#        "targets/x86_64-linux/lib/libcurand.so",
#        "targets/x86_64-linux/lib/libcurand.so.10",
#        "targets/x86_64-linux/lib/libcurand.so.10.3.3.141",
#    ],
#    deps = [
#        "headers",
#    ],
#)

#cc_library(
#    name = "cusolver",
#    srcs = [
#        "targets/x86_64-linux/lib/libcusolver.so",
#        "targets/x86_64-linux/lib/libcusolver.so.11",
#        "targets/x86_64-linux/lib/libcusolver.so.11.4.5.107",
#    ],
#    deps = [
#        "headers",
#    ],
#)

#cc_library(
#    name = "cusparse",
#    srcs = [
#        "targets/x86_64-linux/lib/libcusparse.so",
#        "targets/x86_64-linux/lib/libcusparse.so.12",
#        "targets/x86_64-linux/lib/libcusparse.so.12.2.0.106",
#    ],
#    deps = [
#        "headers",
#    ],
#)

#cc_library(
#    name = "nvToolsExt",
#    srcs = [
#        "targets/x86_64-linux/lib/libnvToolsExt.so",
#        "targets/x86_64-linux/lib/libnvToolsExt.so.1",
#        "targets/x86_64-linux/lib/libnvToolsExt.so.1.0.0",
#    ],
#    deps = [
#        "headers",
#    ],
#)

#cc_library(
#    name = "nvrtc",
#    srcs = [
#        "targets/x86_64-linux/lib/libnvrtc.so",
#        "targets/x86_64-linux/lib/libnvrtc.so.12",
#        "targets/x86_64-linux/lib/libnvrtc.so.12.2.105",
#    ],
#    deps = [
#        "headers",
#    ],
#)

#cc_library(
#    name = "npp",
#    srcs = [
#        "targets/x86_64-linux/lib/libnppc.so",
#        "targets/x86_64-linux/lib/libnppc.so.12",
#        "targets/x86_64-linux/lib/libnppc.so.12.2.0.40",
#        "targets/x86_64-linux/lib/libnppig.so",
#        "targets/x86_64-linux/lib/libnppig.so.12",
#        "targets/x86_64-linux/lib/libnppig.so.12.2.0.40",
#    ],
#    deps = [
#        ":headers",
#    ],
#)

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
