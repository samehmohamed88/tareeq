workspace(name = "tareeq")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("//third_party:repos.bzl", "initialize_third_party")

initialize_third_party()

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
