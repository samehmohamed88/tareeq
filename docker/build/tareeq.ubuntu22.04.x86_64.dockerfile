# Start from the Isaac ROS base image
FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_bcf535ea3b9d16a854aaeb1701ab5a86

# Setup non-root admin user
ARG USERNAME=admin
ARG USER_UID=1001
ARG USER_GID=1001

ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=0

# Install prerequisites including packages from first Dockerfile
RUN apt-get update && apt-get install -y \
    sudo \
    udev \
    software-properties-common \
    apt-transport-https \
    bash-completion \
    build-essential \
    ca-certificates \
    clang-format \
    cmake \
    curl \
    git \
    gnupg2 \
    iputils-ping \
    locales \
    lsb-release \
    rsync \
    wget \
    vim \
    unzip \
    mlocate \
    libgoogle-glog-dev \
    libasio-dev \
    libbullet-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    python3-opencv \
    v4l-utils \
    mesa-utils \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libv4l-dev \
    kmod \
    nvv4l2 \
    libgeographic-dev \
    ros-humble-geographic-info \
    ros-humble-nmea-msgs \
    ros-humble-robot-localization \
    ros-humble-xacro \
&& apt-get clean \
&& rm -rf /var/lib/apt/lists/*

# Python essentials and customizations
RUN apt-get update \
    && apt-get install -y python3-pip python3-pybind11 python3-pytest-cov \
    && python3 -m pip install -U pip \
    && python3 -m pip install -U \
        argcomplete \
        autopep8 \
        flake8==4.0.1 \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        onnx \
        pytest-repeat \
        pytest-rerunfailures \
        pytest \
        pydocstyle \
        scikit-learn \
        torch \
        torchvision \
        torchaudio \
        paho-mqtt \
        protobuf==3.20.1 \
        gpustat==0.6.0 \
        mailcap-fix \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user
RUN if [ $(getent passwd ${USERNAME}) ]; then \
        usermod -l ${USERNAME} -u ${USER_UID} -m -d /home/${USERNAME} ; \
    else \
        groupadd --gid ${USER_GID} ${USERNAME} ; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} ; \
    fi \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod -aG dialout,video,plugdev,sudo ${USERNAME}

# Set environment variables
ENV USERNAME=${USERNAME}
ENV USER_GID=${USER_GID}
ENV USER_UID=${USER_UID}
ENV PATH="${PATH}:/opt/nvidia/tao"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/tritonserver/lib"
ENV USER=${USERNAME}

# Copy scripts
RUN mkdir -p /usr/local/bin/scripts
COPY scripts/*entrypoint.sh /usr/local/bin/scripts/
RUN  chmod +x /usr/local/bin/scripts/*.sh

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY middleware_profiles/*profile.xml /usr/local/share/middleware_profiles/

COPY scripts/install-zed-x86_64.sh /opt/zed/install-zed-x86_64.sh
COPY scripts/install-zed-aarch64.sh /opt/zed/install-zed-aarch64.sh

RUN sudo chmod +x /opt/zed/install-zed-x86_64.sh
RUN sudo chmod +x /opt/zed/install-zed-aarch64.sh

RUN if [ "$(uname -m)" = "x86_64" ]; then \
    /opt/zed/install-zed-x86_64.sh; \
  else \
    /opt/zed/install-zed-aarch64.sh; \
  fi

# Revert to root user
USER root

# Final clean-up
RUN rm -rf /var/lib/apt/lists/* \
    && apt-get clean

USER ${USERNAME}