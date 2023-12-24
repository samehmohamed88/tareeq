""" module to load cuda library """

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# buildifier: disable=unnamed-macro
def repo():
    native.new_local_repository(
        name = "cuda",
        build_file = "//third_party/cuda:cuda.BUILD",
        path = "/usr/local/cuda-12.1",
    )
