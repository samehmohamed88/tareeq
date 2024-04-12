#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

docker build --build-arg USERID=$UID -t sameh4/tareeq:nvidia_isaac_ros_zed_ -f tareeq.ubuntu22.04.x86_64.dockerfile .
