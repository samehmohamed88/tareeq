load("//third_party/cuda:workspace.bzl", cuda = "repo")
load("//third_party/opencv:workspace.bzl", opencv = "repo")
load("//third_party/libtorch:workspace.bzl", libtorch = "repo")
load("//third_party/cudnn:workspace.bzl", cudnn8 = "repo")
load("//third_party/tensorrt:workspace.bzl", tensorrt = "repo")
load("//third_party/rules_ros2:workspace.bzl", rules_ros2 = "repo")

def initialize_third_party():
    cuda()
    opencv()
    libtorch()
    cudnn8()
    tensorrt()
    rules_ros2()
