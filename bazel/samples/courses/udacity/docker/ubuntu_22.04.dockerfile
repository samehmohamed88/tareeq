FROM ubuntu:22.04

ARG USERID=$USERID
ARG USERNAME=nav

ENV DEBIAN_FRONTEND noninteractive    # export DEBIAN_FRONTEND="noninteractive"

RUN apt-get -y update \
  && apt-get -y install git \
                        cmake \
                        build-essential \
                        clang \
                        sudo \
                        wget \
                        libxtst6 \
                        libxt6 \
                        nano \
                        vim \
                        python3-dev \
                        python3-pip \
                        python3-venv \
                        libeigen3-dev \
  && groupadd --gid $USERID $USERNAME \
  && useradd -s /bin/bash --uid $USERID --gid $USERID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && sudo usermod -a -G root,dialout,adm,cdrom,sudo,audio,dip,video,plugdev $USERNAME \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && wget https://github.com/bazelbuild/bazelisk/releases/download/v1.19.0/bazelisk-linux-amd64 \
  && mv bazelisk-linux-amd64 /usr/local/bin/bazel \
  && chmod +x /usr/local/bin/bazel

RUN git clone https://github.com/rogersce/cnpy.git \
  && cd cnpy && mkdir "build" && cd build && cmake ../ && make -j8 && make install

RUN apt-get -y update \
    && apt-get -y install golang \
    && go install github.com/bazelbuild/buildtools/buildifier@latest \
    && mv /root/go/bin/buildifier /usr/local/bin/

RUN apt update && apt install -y openssh-server

EXPOSE 22

ENTRYPOINT service ssh start && bash
