#!/bin/bash
set -xe

# Python essentials and customizations
apt-get update \
  && apt-get install -y python3-pip python3-pybind11 python3-pytest-cov ninja-build \
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
