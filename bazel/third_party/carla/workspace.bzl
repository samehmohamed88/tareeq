""" module to load carla client library """

def repo():
    native.new_local_repository(
        name = "carla",
        build_file = "//third_party/carla:carla.BUILD",
        path = "/usr/local/libcarla-0.9.15",
    )
