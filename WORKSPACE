workspace(name = "nav")
#
#new_local_repository(
#    name = "third_party_prebuilt_libs",
#    build_file_content = "",
#    path = "third_party/binaries",
#)
#
#load("@third_party_prebuilt_libs//:repos.bzl", "load_prebuild_repos")
#
#load_prebuild_repos()

load("//third_party:repos.bzl", "initialize_third_party")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

initialize_third_party()

load("@com_github_mvukov_rules_ros2//repositories:repositories.bzl", "ros2_repositories")

ros2_repositories()

load("@com_github_mvukov_rules_ros2//repositories:deps.bzl", "ros2_deps")

ros2_deps()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

python_register_toolchains(
    name = "rules_ros2_python",
    python_version = "3.10",
)

load("@com_github_mvukov_rules_ros2//repositories:pip_annotations.bzl", "PIP_ANNOTATIONS")
load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_ros2_python//:defs.bzl", python_interpreter_target = "interpreter")

pip_parse(
    name = "rules_ros2_pip_deps",
    annotations = PIP_ANNOTATIONS,
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()
