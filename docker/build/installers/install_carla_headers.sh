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

export PATH="$(dirname "${BASH_SOURCE[0]}")"/llvm-project-llvmorg-10.0.1/build/bin:$PATH \
  && wget https://github.com/carla-simulator/carla/archive/refs/tags/0.9.15.tar.gz \
  && tar -xf 0.9.15.tar.gz \
  && cd carla-0.9.15/Examples/CppClient \
  && make run
