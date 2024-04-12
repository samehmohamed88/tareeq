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

VERSION_X86_64="nvidia_isaac_ros_zed_"

info "Remove existing Tareeq Development container ..."
remove_container_if_exists ${DEV_CONTAINER}

info "Determine whether host GPU is available ..."
determine_gpu_use_host
info "USE_GPU_HOST: ${USE_GPU_HOST}"

volumes="${volumes} -v /media:/media \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /etc/localtime:/etc/localtime:ro \
    -v /usr/src:/usr/src \
    -v /lib/modules:/lib/modules \
    -v ${CURR_DIR}/../../:/tareeq
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

${DOCKER_RUN_CMD} -itd \
    --privileged \
    --name "${DEV_CONTAINER}" \
    --label "owner=${tareeqUSER}" \
    -e DISPLAY="${display}" \
    -e DOCKER_USER="${user}" \
    -e USER="${user}" \
    -e DOCKER_USER_ID="${uid}" \
    -e DOCKER_GRP="${group}" \
    -e DOCKER_GRP_ID="${gid}" \
    -e DOCKER_IMG="${DEV_IMAGE}" \
    -e USE_GPU_HOST="${USE_GPU_HOST}" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
    ${volumes} \
    --net host \
    -w /tareeq \
    --add-host "${DEV_INSIDE}:127.0.0.1" \
    --add-host "${local_host}:127.0.0.1" \
    --hostname "${DEV_INSIDE}" \
    --shm-size "${SHM_SIZE}" \
    --pid=host \
    -v /dev/null:/dev/raw1394 \
    "${DEV_IMAGE}" \
    /bin/bash
    # --gpus all \
