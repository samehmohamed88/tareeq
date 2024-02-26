""" module to load cuda library """

def repo():
    native.new_local_repository(
        name = "sfml",
        build_file = "//third_party/sfml:sfml.BUILD",
        path = "/usr/local/SFML-2.6.1",
    )
