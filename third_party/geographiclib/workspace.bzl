"""Loads the ros2_angles library"""

def repo():
    native.new_local_repository(
        name = "geographiclib",
        build_file = "//third_party/geographiclib:geographiclib.BUILD",
        path = "/usr/local/include/GeographicLib",
    )
