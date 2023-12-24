load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

genrule(
    name = "subaru_global_can_dbc",
    srcs = ["data/subaru_global_2017_generated.dbc"],
    outs = ["SubaruGlobalCANDBC.h"],
    cmd = "(echo 'constexpr char subaruGlobalCANDBC[] = R\"delimiter(' && cat $(SRCS) && echo ')delimiter\";' ) > $@",
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
        "CANDBCMessage.cc",
        "CANDBCMessageSchema.cc",
        "CANDBCSignalSchema.cc",
    ],
    hdrs = [
        "CANDBC.h",
        "CANDBCMessage.h",
        "CANDBCMessageSchema.h",
        "CANDBCSignalSchema.h",
        ":subaru_global_can_dbc",
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

#cc_library(
#    name = "comma_ai_can_interface_simple_deque",
#    hdrs = [
#        "CommaAICANInterfaceWithSimplyDeque.h",
#    ],
#    visibility = ["//nav/can_client:__subpackages__"],
#    deps = [
#        ":can_database",
#        ":device_enum_classes",
#        "@boost",
#    ],
#    alwayslink = True,
#)

#cc_library(
#    name = "comma_ai_can_interface_boost_buffer",
#    hdrs = [
#        "CommaAICANInterfaceWithBoostBuffer.h",
#    ],
#    visibility = ["//nav/can_client:__subpackages__"],
#    deps = [
#        ":can_database",
#        ":device_enum_classes",
#        "@boost",
#    ],
#    alwayslink = True,
#)

cc_library(
    name = "can_exception",
    hdrs = [
        "exceptions/CANCloseException.h",
        "exceptions/CANException.h",
        "exceptions/CANInitException.h",
        "exceptions/InvalidSocketException.h",
    ],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [],
)

cc_library(
    name = "socket_can_interface",
    hdrs = ["SocketCANInterface.h"],
    visibility = ["//nav/can_client:__subpackages__"],
)

cc_library(
    name = "socket_can_message",
    srcs = ["SocketCANMessage.cc"],
    hdrs = ["SocketCANMessage.h"],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [":can_database"],
)

cc_library(
    name = "socket_can_inteface_impl",
    srcs = ["SocketCANInterfaceImpl.cc"],
    hdrs = ["SocketCANInterfaceImpl.h"],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        ":can_exception",
        ":socket_can_interface",
    ],
)

cc_library(
    name = "socket_can_device",
    hdrs = ["SocketCANDevice.h"],
    visibility = ["//nav/can_client:__subpackages__"],
    deps = [
        ":socket_can_message",
        "//cyber/common:log",
    ],
)

#cc_library(
#    name = "buffered_can_client",
#    hdrs = [
#        "CANClient.h",
#    ],
#    visibility = ["//nav/can_client:__subpackages__"],
#    deps = [
#        ":can_database",
#        "//cyber/common:log",
#        "@boost",
#        "@units",
#    ],
#    alwayslink = True,
#)
