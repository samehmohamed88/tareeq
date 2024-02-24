cc_binary(
    name = "calculate_iou",
    srcs = [
        "iou.cc",
        "utils.h",
    ],
    data = ["//01_computer_vision/01_choosing_metrics:json_data"],
    deps = [
        "@cnpy",
        "@eigen3",
        "@nlohmann_json//:json",
    ],
)
