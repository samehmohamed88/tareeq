""" module to load zed library """

def repo():
    native.new_local_repository(
        name = "zed",
        build_file = "//third_party/zed:zed.BUILD",
        path = "/usr/local/zed",
    )
