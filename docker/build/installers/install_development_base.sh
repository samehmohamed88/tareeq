#!/bin/bash
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

apt-get -y update \
  && apt-get -y install curl wget git cmake build-essential clang sudo wget libxtst6 libxt6 libeigen3-dev \
                libxrender1 libxrender-dev libxi6 libxi-dev golang libxrandr-dev libxcursor-dev \
                libudev-dev libopenal-dev libflac-dev libvorbis-dev libgl1-mesa-dev libegl1-mesa-dev \
                libdrm-dev libgbm-dev libgtk-4-dev libopencv-dev \
  && ln -s /usr/lib/x86_64-linux-gnu/libFLAC.so.8.3.0 /usr/lib/x86_64-linux-gnu/libFLAC.so.12 \
  && cp -r /usr/include/opencv4 /usr/local/include \
  && cp /usr/lib/x86_64-linux-gnu/libopencv_* /usr/local/lib \
  && groupadd --gid $USERID $USERNAME \
  && useradd -s /bin/bash --uid $USERID --gid $USERID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && sudo usermod -a -G root,dialout,adm,cdrom,sudo,audio,dip,video,plugdev $USERNAME \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && wget https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/bazelisk-linux-amd64 \
  && mv bazelisk-linux-amd64 /usr/local/bin/bazel \
  && chmod +x /usr/local/bin/bazel \
  && wget https://www.sfml-dev.org/files/SFML-2.6.1-linux-gcc-64-bit.tar.gz \
  && tar -xf SFML-2.6.1-linux-gcc-64-bit.tar.gz \
  && mv SFML-2.6.1 /usr/local/ \
  && rm -rf SFML-2.6.1-linux-gcc-64-bit.tar.gz \
  && go install github.com/bazelbuild/buildtools/buildifier@latest \
  && mv ~/go/bin/buildifier /usr/local/bin \
  && wget https://github.com/gbmhunter/CppLinuxSerial/archive/refs/tags/v2.8.1.tar.gz \
  && tar -xf v2.8.1.tar.gz \
  && cd CppLinuxSerial-2.8.1 \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && make install \
  && cd ../../ \
  && rm -rf v2.8.1.tar.gz  CppLinuxSerial-2.8.1
