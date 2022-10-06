#!/bin/bash
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Install system deps
# Toolchains
sudo apt-get install -y cmake ninja-build npm doxygen git
# VRS, Sophus and Pybind11 dependencies
sudo apt-get install -y \
    libturbojpeg-dev libeigen3-dev \
    libpng-dev liblz4-dev libzstd-dev libxxhash-dev \
    libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-chrono-dev libboost-date-time-dev \
    libgtest-dev libgmock-dev libgoogle-glog-dev \
    libglew-dev libgl1-mesa-dev \
    libwayland-dev libxkbcommon-dev wayland-protocols \
    libpython3-dev python3-pip;
sudo apt-get clean;
# Installing pybind11
sudo pip3 install pybind11[global] numpy pyopengl;

# Install and compile libraries
sh ./install_compile_3rd_party.sh
