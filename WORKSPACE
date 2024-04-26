workspace(name = "tareeq")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

#load("//third_party:repos.bzl", "initialize_third_party")
#
#initialize_third_party()

load("//tools:environ.bzl", "environment_repository")

environment_repository(
    name = "tareeq_bazel_environ",
    envvars = ["ROS2_DISTRO_PREFIX"],
)


http_archive(
    name = "com_google_googletest",
    strip_prefix = "googletest-1.14.0",
    urls = ["https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz"],
)


# Boost
# Famous C++ library that has given rise to many new additions to the C++ Standard Library
# Makes @boost available for use: For example, add `@boost//:algorithm` to your deps.
# For more, see https://github.com/nelhage/rules_boost and https://www.boost.org
http_archive(
    name = "com_github_nelhage_rules_boost",
    sha256 = "1ef96e5c4c7c05024e60dc5927f7c1f39c692530d1a396f0968ce1715cd00df4",
    strip_prefix = "rules_boost-e72eb259976357f6e82f4d74d74a7c12d1c3776d",
    url = "https://github.com/nelhage/rules_boost/archive/e72eb259976357f6e82f4d74d74a7c12d1c3776d.tar.gz",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

boost_deps()

# Use the ROS 2 bazel rules
local_repository(
    name = "bazel_ros2_rules",
    path = "./bazel_ros2_rules",
)

load("@bazel_ros2_rules//deps:defs.bzl", "add_bazel_ros2_rules_dependencies")

add_bazel_ros2_rules_dependencies()

load(
    "@bazel_ros2_rules//ros2:defs.bzl",
    "ros2_archive",
    "ros2_local_repository",
)

load(
    "@tareeq_bazel_environ//:environ.bzl",
    "ROS2_DISTRO_PREFIX",
)

# Please keep this list sorted
ROS2_PACKAGES = [
    "action_msgs",
    "builtin_interfaces",
    "console_bridge_vendor",
    "rclcpp",
    "rclcpp_action",
    "rclpy",
    "ros2cli",
    "ros2cli_common_extensions",
    "rosbag2",
    "rosidl_default_generators",
    "tf2_py",
] + [
    # These are possible RMW implementations. Uncomment one and only one to
    # change implementations
    "rmw_cyclonedds_cpp",
    # "rmw_fastrtps_cpp",
]

RESOLVED_PREFIX = (
    ROS2_DISTRO_PREFIX if ROS2_DISTRO_PREFIX else "/opt/ros/humble"
)

ros2_local_repository(
    name = "ros2",
    include_packages = ROS2_PACKAGES,
    workspaces = [RESOLVED_PREFIX, "workspaces/isaac_ros-dev"],
)
