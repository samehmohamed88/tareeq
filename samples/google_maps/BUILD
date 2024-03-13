load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_doc")

package(default_visibility = ["//visibility:public"])

rust_binary(
    name = "helloworld",
    srcs = ["helloworld.rs"],
)

#load("@crate_index_cargo_workspace//:defs.bzl", "all_crate_deps")
#load("@rules_rust//rust:defs.bzl", "rust_binary")
#
#package(default_visibility = ["//visibility:public"])
#
#rust_binary(
#    name = "number_printer",
#    srcs = ["src/popup_window"],
#    edition = "2018",
#    deps = [
#        "//cargo_workspace/printer",
#    ] + all_crate_deps(),
#)
