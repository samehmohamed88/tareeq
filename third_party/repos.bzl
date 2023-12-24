load("//third_party/cuda:workspace.bzl", cuda = "repo")
load("//third_party/libtorch:workspace.bzl", libtorch = "repo")
load("//third_party/rules_ros2:workspace.bzl", rules_ros2 = "repo")

def initialize_third_party():
    cuda()
    libtorch()
    rules_ros2()
