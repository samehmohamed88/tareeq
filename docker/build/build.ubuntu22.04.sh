#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

docker build --build-arg USERID=$UID -t sameh4/tareeq:nvidia-isaac-ros.x86_64.04.25.2024 -f tareeq.nvidia-isaac-ros.x86_64.dockerfile .
