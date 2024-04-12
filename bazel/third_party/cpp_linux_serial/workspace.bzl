""" module to load CPP Linux Serial library """

def repo():
    native.new_local_repository(
        name = "cpp_linux_serial",
        build_file = "//third_party/cpp_linux_serial:cpp_linux_serial.BUILD",
        path = "/usr/local/include",
    )
