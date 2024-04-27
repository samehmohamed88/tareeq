#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

if [ "$(uname -m)" = "x86_64" ]; then \
    BASE_IMAGE=isaac_ros_dev-x86_64
else \
    BASE_IMAGE=isaac_ros_dev-aarch64
fi

docker build \
        --build-arg USERID=$UID \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        -t sameh4/tareeq:nvidia-isaac-ros.x86_64.04.25.2024 \
        -f tareeq.nvidia-isaac-ros.x86_64.dockerfile .
