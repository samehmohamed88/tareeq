# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

apt-get -y update \
  && apt-get -y install git cmake build-essential clang sudo wget libxtst6 libxt6 libeigen3-dev \
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

