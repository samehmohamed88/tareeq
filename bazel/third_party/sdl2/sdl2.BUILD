load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sdl2",
    hdrs = glob(["SDL/**"]),
    includes = ["./SDL"],
    linkopts = [
        "-lSDL2-2.0",
    ],
)
