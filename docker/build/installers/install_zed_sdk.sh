# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

export DEBIAN_FRONTEND=noninteractive \
  && wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.0/ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run \
  && chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run \
  && apt-get install -y keyboard-configuration \
  && apt-get -y install zstd \
  && ./ZED_SDK_Ubuntu22_cuda12.1_v4.0.8.zstd.run -- silent
