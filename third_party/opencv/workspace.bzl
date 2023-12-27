""" module to load opencv library """

def repo():
    native.new_local_repository(
        name = "opencv",
        build_file = "//third_party/opencv:opencv.BUILD",
        path = "/usr/local/include/opencv4",
    )
