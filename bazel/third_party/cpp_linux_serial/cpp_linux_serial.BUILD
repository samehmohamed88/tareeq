load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cpp_linux_serial",
    hdrs = glob(["CppLinuxSerial/*.hpp"]),
    includes = ["CppLinuxSerial"],
    linkopts = [
        "-L/usr/local/lib",
        "-lCppLinuxSerial",
    ],
)
