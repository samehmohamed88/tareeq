workspace(name = "nav")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
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

local_repository(
    name = "rules_cuda",
    path = "third_party/rules_cuda",
)

load("@rules_cuda//cuda:repositories.bzl", "register_detected_cuda_toolchains", "rules_cuda_dependencies")

rules_cuda_dependencies()

register_detected_cuda_toolchains()

# Boost
# Famous C++ library that has given rise to many new additions to the C++ Standard Library
# Makes @boost available for use: For example, add `@boost//:algorithm` to your deps.
# For more, see https://github.com/nelhage/rules_boost and https://www.boost.org
http_archive(
    name = "com_github_nelhage_rules_boost",
    sha256 = "1ef96e5c4c7c05024e60dc5927f7c1f39c692530d1a396f0968ce1715cd00df4",
    strip_prefix = "rules_boost-96e9b631f104b43a53c21c87b01ac538ad6f3b48",
    url = "https://github.com/nelhage/rules_boost/archive/e72eb259976357f6e82f4d74d74a7c12d1c3776d.tar.gz",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

boost_deps()
