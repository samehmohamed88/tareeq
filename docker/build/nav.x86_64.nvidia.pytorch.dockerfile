FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04

ARG USERID=1000
ARG USERNAME=nav

ENV DEBIAN_FRONTEND noninteractive    # export DEBIAN_FRONTEND="noninteractive"

RUN mkdir -p /opt/nav

COPY installers /opt/nav/installers

RUN bash /opt/nav/installers/install_development_base.sh

USER $USERNAME

RUN sudo bash /opt/nav/installers/install_python.sh
RUN bash /opt/nav/installers/install_pytorch.sh
RUN sudo bash /opt/nav/installers/install_zed_sdk.sh

RUN sudo apt-get -y clean \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /nav
