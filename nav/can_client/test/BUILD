load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "mock_lib_usb_device",
    srcs = ["mocks/MockLibUsbDevice.h"],
    deps = [
        "@com_google_googletest//:gtest",
        "@units",
    ],
)

cc_test(
    name = "test_usb_device",
    srcs = ["UsbDeviceTest.cc"],
    deps = [
        "//nav/can_client:device_enum_classes",
        "//nav/can_client:lib_usb_device_interface",
        "//nav/can_client:usb_device",
        "//nav/can_client/test:mock_lib_usb_device",
        "@com_google_googletest//:gtest",
        "@units",
    ],
)

cc_test(
    name = "test_can_dbc",
    srcs = ["CANDBCTest.cc"],
    deps = [
        "//nav/can_client:can_database",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "test_comma_ai_device",
    srcs = ["CommaAICANAdapterTest.cc"],
    deps = [
        "//nav/can_client:comma_ai_can_adapter",
        "//nav/can_client:device_enum_classes",
        "//nav/can_client:lib_usb_device",
        "//nav/can_client:lib_usb_device_interface",
        "//nav/can_client:usb_device",
        "@com_google_googletest//:gtest",
        "@units",
    ],
)

cc_test(
    name = "test_comma_ai_can_interface_simple_deque",
    srcs = ["CommaAICANInterfaceWithSimpleDequeTest.cc"],
    deps = [
        "//nav/can_client:comma_ai_can_interface_simple_deque",
        "//nav/can_client:device_enum_classes",
        "//nav/can_client:lib_usb_device",
        "//nav/can_client:lib_usb_device_interface",
        "//nav/can_client:usb_device",
        "@com_google_googletest//:gtest",
        "@units",
    ],
)
