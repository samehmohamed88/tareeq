FROM pytorch/pytorch:2.2.1-cuda12.1-cudnn8-devel
#FROM ubuntu:22.04

ARG USERID=$USERID
ARG USERNAME=tareeq

ENV DEBIAN_FRONTEND noninteractive    # export DEBIAN_FRONTEND="noninteractive"

RUN mkdir -p /opt/tareeq

COPY installers /opt/tareeq/installers

RUN bash /opt/tareeq/installers/install_development_base.sh

# COPY libcarla-client-library-0.9.15.tar.gz /usr/local/
# RUN tar -xf /usr/local/libcarla-client-library-0.9.15.tar.gz -C /usr/local/
# RUN rm /usr/local/libcarla-client-library-0.9.15.tar.gz

USER $USERNAME

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

RUN sudo bash /opt/tareeq/installers/install_python.sh
RUN sudo bash /opt/tareeq/installers/install_zed_sdk.sh

RUN sudo apt-get -y clean \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /tareeq
