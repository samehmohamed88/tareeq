""" module to load cudnn library """

def repo():
    native.new_local_repository(
        name = "cudnn8",
        build_file = "//third_party/cudnn:cudnn.BUILD",
        path = "/usr/local/cudnn-8.9.7.29_cuda12",
    )
