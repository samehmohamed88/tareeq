load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

genrule(
    name = "subaru_global_can_dbc",
    srcs = ["data/subaru_global_2017_generated.dbc"],
    outs = ["SubaruGlobalCANDBC.h"],
    cmd = "(echo 'constexpr char subaruGlobalCANDBC[] = R\"delimiter(' && cat $(SRCS) && echo ')delimiter\";' ) > $@",
)

cc_binary(
    name = "libpanda_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":panda_component_inner"],
)

cc_library(
    name = "device_enum_classes",
    hdrs = ["DeviceInfo.h"],
    visibility = ["//nav/can_client:__subpackages__"],
    alwayslink = True,
)

cc_library(
    name = "can_database",
    srcs = [
        "CANDBC.cc",
    ],
    hdrs = [
        "CANDBC.h",
        ":subaru_global_can_dbc",
    ],
    alwayslink = True,
)

cc_library(
    name = "comma_ai_can_adapter",
    hdrs = [
        "CommaAICANInterface.h",
    ],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        ":can_database",
        ":device_enum_classes",
        "@boost",
    ],
    alwayslink = True,
)

cc_library(
    name = "panda_component_inner",
    srcs = ["panda_component.cc"],
    hdrs = ["panda_component.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "usb_can_adapter",
    srcs = ["UsbCANAdapter.cc"],
    hdrs = ["UsbCANAdapter.h"],
    visibility = ["//visibility:private"],
    deps = [
        "@units",
    ],
    alwayslink = True,
)

cc_library(
    name = "panda_api",
    srcs = ["CANClient.cc"],
    hdrs = ["CANClient.h"],
    visibility = ["//visibility:private"],
    deps = [
        ":panda_usb_handler",
    ],
    alwayslink = True,
)

cc_library(
    name = "lib_usb_device_interface",
    hdrs = ["LibUsbDeviceInterface.h"],
    linkopts = ["-lusb-1.0"],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        "@units",
    ],
    alwayslink = True,
)

cc_library(
    name = "lib_usb_device",
    srcs = ["LibUsbDevice.cc"],
    hdrs = ["LibUsbDevice.h"],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        ":lib_usb_device_interface",
        "@units",
    ],
    alwayslink = True,
)

cc_library(
    name = "usb_device",
    hdrs = ["UsbDevice.h"],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        ":device_enum_classes",
        "//cyber/common:log",
        "@units",
    ],
    alwayslink = True,
)

filegroup(
    name = "conf",
    srcs = [
        ":panda.dag",
        ":panda.launch",
    ],
)

cc_library(
    name = "comma_ai_can_interface_simple_deque",
    hdrs = [
        "CommaAICANInterfaceWithSimplyDeque.h",
    ],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        ":can_database",
        ":device_enum_classes",
        "@boost",
    ],
    alwayslink = True,
)

cc_library(
    name = "comma_ai_can_interface_boost_buffer",
    hdrs = [
        "CommaAICANInterfaceWithBoostBuffer.h",
    ],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        ":can_database",
        ":device_enum_classes",
        "@boost",
    ],
    alwayslink = True,
)

#install(
#    name = "install",
#    data = [
#        ":conf",
#    ],
#    # We want to install channel_*_writer into runtime_dest
#    # rather than bin/
#    runtime_dest = "cyber/examples/common_component_example",
#    targets = [
#        ":channel_prediction_writer",
#        ":channel_test_writer",
#        ":libcommon_component_example.so",
#    ],
#)

cpplint()
