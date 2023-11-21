load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_binary(
    name = "ReadCANMessages",
    srcs = ["ReadCANMessages.cc"],
    linkopts = ["-pthread"],
    deps = [
        "//nav/can_client:comma_ai_can_adapter",
        "//nav/can_client:device_enum_classes",
        "//nav/can_client:lib_usb_device",
        "//nav/can_client:lib_usb_device_interface",
        "//nav/can_client:usb_device",
        "@boost",
    ],
)
