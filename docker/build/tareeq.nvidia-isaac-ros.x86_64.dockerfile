# FROM isaac_ros_dev-aarch64
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Setup non-root admin user
ARG USERNAME=admin
ARG USER_UID=${UID}
ARG USER_GID=${UID}

ENV USERNAME=${USERNAME}
ENV USER_GID=${USER_GID}
ENV USER_UID=${USER_UID}

USER root

RUN mkdir -p /opt/tareeq
COPY installers /opt/tareeq/installers

#RUN /opt/tareeq/installers/install-gcc11.sh

RUN if [ "$(uname -m)" = "x86_64" ]; then \
        /opt/tareeq/installers/install-bazel-x86_64.sh; \
    else \
        /opt/tareeq/installers/install-bazel-aarch64.sh; \
    fi

# Install prerequisites
RUN apt-get update && apt-get install -y \
        sudo \
        udev \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

# Reuse triton-server user as 'admin' user if exists
RUN if [ $(getent group triton-server) ]; then \
        groupmod -o --gid ${USER_GID} -n ${USERNAME} triton-server ; \
        usermod -l ${USERNAME} -u ${USER_UID} -m -d /home/${USERNAME} triton-server ; \
        mkdir -p /home/${USERNAME} ; \
        sudo chown ${USERNAME}:${USERNAME} /home/${USERNAME} ; \
    fi

# Create the 'admin' user if not already exists
RUN if [ ! $(getent passwd ${USERNAME}) ]; then \
        groupadd --gid ${USER_GID} ${USERNAME} ; \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} ; \
    fi

# Update 'admin' user
RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && adduser ${USERNAME} video && adduser ${USERNAME} plugdev && adduser ${USERNAME} sudo && adduser ${USERNAME} dialout

RUN apt-get -y update \
      && apt-get install -y \
        python3-toposort python3-dev python-is-python3 gdb lldb \
        ros-humble-rmw-cyclonedds-cpp \
        ros-dev-tools \
        ros-humble-isaac-ros-visual-slam \
        ros-humble-isaac-ros-stereo-image-proc \
        ros-humble-isaac-ros-image-proc \
        ros-humble-foxglove-bridge \
        ninja-build

#ros-humble-nvblox* \

RUN wget https://github.com/fmtlib/fmt/archive/refs/tags/10.2.1.tar.gz \
        && tar -xf 10.2.1.tar.gz \
        && cd fmt-10.2.1 \
        && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE \
        && cmake --build build --config Release -j$(nproc) \
        && cmake --build build --target install -j$(nproc) \
        && cd .. \
        && rm -rf fmt-10.2.1 10.2.1.tar.gz

# Copy scripts
RUN mkdir -p /usr/local/bin/scripts
COPY scripts/*.sh /usr/local/bin/scripts/
RUN  chmod +x /usr/local/bin/scripts/*.sh

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY middleware_profiles/*profile.xml /usr/local/share/middleware_profiles/

WORKDIR /workspace
