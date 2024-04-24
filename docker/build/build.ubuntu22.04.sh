#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

docker build --build-arg USERID=$UID -t sameh4/tareeq:pytorch-2.2.1-cuda12.1-cudnn8-devel.04.24.2024 -f tareeq.ubuntu22.04.x86_64.dockerfile .
