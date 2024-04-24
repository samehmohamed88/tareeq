#!/bin/bash
set -xe

## Setup user
# shellcheck disable=SC2046
if [ $(getent passwd ${USERNAME}) ]; then \
      usermod -l ${USERNAME} -u ${USER_UID} -m -d /home/${USERNAME} ; \
  else \
      groupadd --gid ${USER_GID} ${USERNAME} ; \
      useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} ; \
  fi \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && usermod -aG dialout,video,plugdev,sudo ${USERNAME}
