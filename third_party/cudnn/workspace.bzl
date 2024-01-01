""" module to load cudnn library """

def repo():
    native.new_local_repository(
        name = "cudnn8",
        build_file = "//third_party/cudnn:cudnn.BUILD",
        path = "/usr/local/cuda-12.2",
    )
