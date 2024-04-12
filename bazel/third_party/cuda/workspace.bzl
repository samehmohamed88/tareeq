""" module to load cuda library """

def repo():
    native.new_local_repository(
        name = "cuda",
        build_file = "//third_party/cuda:cuda.BUILD",
        path = "/usr/local/cuda",
    )
