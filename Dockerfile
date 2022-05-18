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

# Start from an ubuntu container
FROM ubuntu:focal

# Get dependencies
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" TZ="America/New_York" apt-get install -y tzdata

# Install build
RUN apt-get install -y cmake ninja-build ccache npm doxygen git;
# Install VRS, Sophus and other dependencies
RUN apt-get install -y \
    libjpeg-dev libeigen3-dev \
    libpng-dev liblz4-dev libzstd-dev libxxhash-dev \
    libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-chrono-dev libboost-date-time-dev \
    libgtest-dev libgmock-dev libgoogle-glog-dev \
    libglew-dev libgl1-mesa-dev \
    libwayland-dev libxkbcommon-dev wayland-protocols \
    libpython3-dev python3-pip; apt-get clean;



# Installing pybind11
RUN pip3 install pybind11 numpy pyopengl;

# Code
ADD ./ /opt/aria_research_kit

# Compile and build and install 3rd_party libs
RUN sh /opt/aria_research_kit/src/scripts/install_compile_3rd_party.sh

# Configure
RUN mkdir /opt/aria_research_kit_Build; cd /opt/aria_research_kit_Build; cmake -DCMAKE_BUILD_TYPE=RELEASE /opt/aria_research_kit/src;

# Build & test
RUN cd /opt/aria_research_kit_Build; make -j ; ctest -j;

# Build python bindings
RUN cd /opt/aria_research_kit/src; pip3 install --global-option=build_ext --global-option="-j8" .;
