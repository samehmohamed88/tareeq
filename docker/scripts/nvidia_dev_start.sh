#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${CURR_DIR}/docker_base.sh"

tareeqUSER="admin"
CACHE_ROOT_DIR="$USER/.cache"

DOCKER_REPO="sameh4/tareeq"
DEV_INSIDE="in-dev-docker"
DEV_CONTAINER_PREFIX='tareeq_dev_'
DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${tareeqUSER}"

VERSION_X86_64="nvidia-isaac-ros.x86_64.04.25.2024"

info "Remove existing Tareeq Development container ..."
remove_container_if_exists ${DEV_CONTAINER}

info "Determine whether host GPU is available ..."
determine_gpu_use_host
info "USE_GPU_HOST: ${USE_GPU_HOST}"

volumes="${volumes} \
    -v ${HOME}/workspace:/workspace
    --volume=${HOME}/CLION/clion-2023.3.2:/home/$tareeqUSER/clion \
    --volume=${HOME}/.dockerConfig/jetbrains/.java/.userPrefs:/home/$tareeqUSER/.java/.userPrefs \
    --volume=${HOME}/.dockerConfig/jetbrains/cache:/home/$tareeqUSER/.cache/JetBrains \
    --volume=${HOME}/.dockerConfig/jetbrains/share:/home/$tareeqUSER/.local/share/JetBrains \
    --volume=${HOME}/.dockerConfig/jetbrains/config:/home/$tareeqUSER/.config/JetBrains \
    --volume=${HOME}/.ideavimrc:/home/$tareeqUSER/.ideavimrc \
    --volume=/media:/media \
    --volume=${HOME}/.fonts:/home/$tareeqUSER/.fonts"

SHM_SIZE="2G"
DEV_IMAGE="${DOCKER_REPO}:${VERSION_X86_64}"

local_host="$(hostname)"
display="${DISPLAY:-:0}"
user="${CUSTOM_USER-$USER}"
uid="${CUSTOM_UID-$(id -u)}"
group="${CUSTOM_GROUP-$(id -g -n)}"
gid="${CUSTOM_GID-$(id -g)}"

DOCKER_RUN_CMD="docker run"

set -x

echo $USE_GPU_HOST

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")

PLATFORM="$(uname -m)"

if [[ $PLATFORM == "aarch64" ]]; then
    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/argus_socket:/tmp/argus_socket")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcufft.so.10")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1")
    DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h")
    DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("-v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli")
    DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v /opt/nvidia/vpi2:/opt/nvidia/vpi2")
    DOCKER_ARGS+=("-v /usr/share/vpi2:/usr/share/vpi2")

    # If jtop present, give the container access
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
        JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
        DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
    fi
fi



BASE_NAME="sameh4/tareeq:nvidia-isaac-ros.x86_64.04.25.2024"
CONTAINER_NAME="nvidia-isaac-ros.x86_64.04.25.2024"

docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v /dev/*:/dev/* \
    -v /etc/localtime:/etc/localtime:ro \
    --name "$CONTAINER_NAME" \
    ${volumes} \
    --runtime nvidia \
    --user="admin" \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir /workspace \
    $@ \
    $BASE_NAME \
    /bin/bash
