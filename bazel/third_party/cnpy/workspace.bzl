""" module to load cnpy library """

def repo():
    native.new_local_repository(
        name = "cnpy",
        build_file = "//third_party/cnpy:cnpy.BUILD",
        path = "/usr/local/include",
    )
