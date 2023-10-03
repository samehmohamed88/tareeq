ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG APOLLO_DIST
ARG GEOLOC
ARG CLEAN_DEPS
ARG INSTALL_MODE

LABEL version="1.2"

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH /opt/apollo/sysroot/bin:$PATH
ENV APOLLO_DIST ${APOLLO_DIST}

COPY installers /opt/apollo/installers
COPY rcfiles /opt/apollo/rcfiles

RUN bash /opt/apollo/installers/install_minimal_environment.sh ${GEOLOC}
RUN bash /opt/apollo/installers/install_cmake.sh
RUN bash /opt/apollo/installers/install_cyber_deps.sh ${INSTALL_MODE}
RUN bash /opt/apollo/installers/install_llvm_clang.sh
RUN bash /opt/apollo/installers/install_qa_tools.sh
RUN bash /opt/apollo/installers/install_visualizer_deps.sh
RUN bash /opt/apollo/installers/install_bazel.sh
RUN bash /opt/apollo/installers/post_install.sh cyber

ENV DEBIAN_FRONTEND=noninteractive
ARG USERNAME=sameh.mohamed
ARG USERID=1001

RUN groupadd --gid $USERID $USERNAME \
  && useradd -s /bin/bash --uid $USERID --gid $USERID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo  \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && sudo usermod -a -G root,dialout,adm,cdrom,sudo,audio,dip,video,plugdev $USERNAME \
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

WORKDIR /apollo
