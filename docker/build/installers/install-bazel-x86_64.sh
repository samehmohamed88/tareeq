#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

apt-get update -y \
    && apt-get -y install golang \
    && wget https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/bazelisk-linux-amd64 \
    && mv bazelisk-linux-amd64 /usr/local/bin/bazel \
    && chmod +x /usr/local/bin/bazel \
    && export GO111MODULE=on \
    && go get github.com/bazelbuild/buildtools/buildifier@latest \
    && mv ~/go/bin/buildifier /usr/local/bin
