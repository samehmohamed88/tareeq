#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    && apt update -y \
    && apt install -y gcc-11 g++-11 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 60 --slave /usr/bin/g++ g++ /usr/bin/g++-11 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 40 --slave /usr/bin/g++ g++ /usr/bin/g++-9


# export CYCLONEDDDS_URI=/workspace/src/tareeq/docker/build/middleware_profiles/cyclone_dds_localhost.xml
