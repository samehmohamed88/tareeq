""" module to load units library """

def repo():
    native.new_local_repository(
        name = "libtorch",
        build_file = "//third_party/libtorch:libtorch.BUILD",
        path = "/usr/local/libtorch",
    )
