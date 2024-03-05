load("@rules_cc//cc:defs.bzl", "cc_library")

AS_NEEDED = [
    "lib/libbackend_with_compiler.so",
    "lib/libjitbackend_test.so",
    "lib/libc10_cuda.so",
    "lib/libc10.so",
    "lib/libcaffe2_nvrtc.so",
    "lib/libtorchbind_test.so",
    "lib/libtorch_cpu.so",
    "lib/libtorch_cuda_cpp.so",
    "lib/libtorch_cuda_cu.so",
    "lib/libtorch_global_deps.so",
    "lib/libtorch.so",
    "lib/libshm.so",
    "lib/libgomp-52f2fd74.so.1",
    "lib/libcudart-9335f6a2.so.12",
    "lib/libnvrtc-b51b459d.so.12",
    "lib/libnvToolsExt-847d78f2.so.1",
    "lib/libcublasLt-f97bfc2c.so.12",
    "lib/libcublas-37d11411.so.12",
]

NO_AS_NEEDED = [
    "lib/libtorch_cuda.so",
]

LIBTORCH = AS_NEEDED

LIBTORCH_GPU = AS_NEEDED + NO_AS_NEEDED

cc_library(
    name = "libtorch",
    srcs = LIBTORCH,
    hdrs = glob(["include/**/*.h"]),
    includes = [
        "include",
        "include/TH",
        "include/THC",
        "include/torch/csrc/api/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        #"@av_toolchain//:gomp",
        "@cuda//:cublas",
        "@cuda//:cudart",
        "@cuda//:cufft",
        "@cuda//:curand",
        "@cuda//:cusolver",
        "@cuda//:cusparse",
        "@cuda//:nvToolsExt",
        "@cuda//:nvrtc",
    ],
)

cc_library(
    name = "libtorch_gpu",
    srcs = LIBTORCH_GPU,
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.cuh",
    ]),
    includes = [
        "include",
        "include/ATen",
        "include/TH",
        "include/THC",
        "include/caffe2",
        "include/torch/csrc/api/include",
    ],
    linkopts = ["-Wl,--no-as-needed -ltorch_cuda -lcusparse -lcurand -lcusolver -lnvToolsExt -lcufft -lcublas -lcublasLt"],  # Force ld to link the cuda shared libraries even if not used as required by Pytorch
    visibility = ["//visibility:public"],
    deps = [
        #"@av_toolchain//:gomp",
        "@cuda//:cublas",
        "@cuda//:cudart",
        "@cuda//:cufft",
        "@cuda//:curand",
        "@cuda//:cusolver",
        "@cuda//:cusparse",
        "@cuda//:nvToolsExt",
        "@cuda//:nvrtc",
        "@cudnn",
    ],
)

cc_library(
    name = "torch_cpu",
    srcs = [
        "lib/libc10.so",
        "lib/libcudart-9335f6a2.so.12",
        "lib/libgomp-52f2fd74.so.1",
        "lib/libshm.so",
        "lib/libtorch.so",
        "lib/libtorch_cpu.so",
        "lib/libtorch_global_deps.so",
    ],
    hdrs = glob(["include/**/*.h"]),
    includes = [
        "include",
        "include/TH",
        "include/THC",
        "include/torch/csrc/api/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        #"@av_toolchain//:gomp",
        #"@cuda//:cudart",
    ],
)

filegroup(
    name = "libtorch_files",
    srcs = LIBTORCH,
    visibility = ["//visibility:public"],
)
