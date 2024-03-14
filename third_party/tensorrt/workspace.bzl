""" module to load cudnn library """

def repo():
    native.new_local_repository(
        name = "tensorrt8",
        build_file = "//third_party/tensorrt:tensorrt.BUILD",
        path = "/usr/local/tensorRT-8.6.1.6",
    )
