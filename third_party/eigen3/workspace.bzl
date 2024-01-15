""" module to load cnpy library """

def repo():
    native.new_local_repository(
        name = "eigen3",
        build_file = "//third_party/eigen3:eigen3.BUILD",
        path = "/usr/include/eigen3",
    )
