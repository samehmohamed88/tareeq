# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

docker build --build-arg USERID=$UID -t nav:ubuntu_2204_x86_64.nvidia.pytorch.01.15.2024  -f nav.x86_64.nvidia.pytorch.dockerfile .
