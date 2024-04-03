""" module to load cnpy library """

def repo():
    native.new_local_repository(
        name = "sdl2",
        build_file = "//third_party/sdl2:sdl2.BUILD",
        path = "/usr/include",
    )
