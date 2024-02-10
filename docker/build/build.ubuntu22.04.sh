#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

docker build --build-arg USERID=$UID -t tareeq:ubuntu_2204_x86_64.02.08.2024 -f tareeq.ubuntu22.04.x86_64.dockerfile .
