#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# install clang-10 from source
# wget https://github.com/llvm/llvm-project/archive/refs/tags/llvmorg-10.0.1.tar.gz \
#   && tar -xf llvmorg-10.0.1.tar.gz \
#   && cd llvm-project-llvmorg-10.0.1 \
#   && mkdir build \
#   && cd build \
#   && cmake -DLLVM_ENABLE_PROJECTS=clang -G "Unix Makefiles" ../llvm \
#   && make clang -j`nproc`
  
# cd "$(dirname "${BASH_SOURCE[0]}")"

# export PATH="$(dirname "${BASH_SOURCE[0]}")"/llvm-project-llvmorg-10.0.1/build/bin:$PATH \
#   && wget https://github.com/carla-simulator/carla/archive/refs/tags/0.9.15.tar.gz \
#   && tar -xf 0.9.15.tar.gz \
#   && cd carla-0.9.15/Examples/CppClient \
#   && make run

apt-get -y update \
  && apt-get -y install wget software-properties-common \
  && add-apt-repository ppa:ubuntu-toolchain-r/test \
  && wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add \
  && apt-add-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal main" \
  && apt-get -y update \
  && apt-get -y install build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 \
                     python python-dev python3-dev python3-pip libpng-dev libtiff5-dev \
                     libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git \
  && update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 \
  && update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180 \
  && pip3 install --upgrade pip \
  && pip install --upgrade pip \
  && pip install --user setuptools 
  && pip3 install --user -Iv setuptools==47.3.1 \
  && pip install --user distro \
  && pip3 install --user distro \
  && pip install --user wheel \
  && pip3 install --user wheel auditwheel
