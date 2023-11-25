load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_binary(
    name = "read_can_messages",
    srcs = ["ReadCANMessages.cc"],
    linkopts = ["-pthread"],
    deps = [
        "//nav/can_client:can_database",
        "//nav/can_client:comma_ai_can_interface_simple_deque",
        "//nav/can_client:device_enum_classes",
        "//nav/can_client:lib_usb_device",
        "//nav/can_client:lib_usb_device_interface",
        "//nav/can_client:usb_device",
        "@boost",
    ],
)
