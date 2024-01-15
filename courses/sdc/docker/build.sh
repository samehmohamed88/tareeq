# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

docker build --build-arg USERID=$UID -t sameh4/udacity_sdc:ubuntu_22.04   -f ubuntu_22.04.dockerfile .
