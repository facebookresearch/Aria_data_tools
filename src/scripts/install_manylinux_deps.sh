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
yum install -y cmake git gtest-devel \
    lz4-devel libzstd-devel xxhash-devel libpng-devel \
    python3-pip

# Installing pybind11
pip3 install pybind11[global]

# Install and compile libraries
thread=$(nproc)

# Build Fmt
cd /tmp; git clone https://github.com/fmtlib/fmt.git -b 8.1.1 \
    && cd fmt \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE -DFMT_TEST=OFF .; make -j$thread install; rm -rf /tmp/fmt;

cd /tmp && git clone https://github.com/USCiLab/cereal.git -b v1.3.2 \
    && cd cereal \
    && cmake -DSKIP_PORTABILITY_TEST=1 -DJUST_INSTALL_CEREAL=ON .;make -j4 install; rm -rf /tmp/cereal;

cd /tmp && git clone https://github.com/libjpeg-turbo/libjpeg-turbo.git -b 2.1.4 \
    && cd libjpeg-turbo \
    && cmake -DCMAKE_BUILD_TYPE=Release -DWITH_JPEG8=1 -DCMAKE_INSTALL_DEFAULT_PREFIX=/usr .;make -j$thread install; rm -rf /tmp/libjpeg-turbo;

cd /tmp && git clone --recursive https://github.com/boostorg/boost.git -b boost-1.81.0 \
    && cd boost \
    && ./bootstrap.sh \
    && ./b2 --with-system --with-filesystem --with-thread --with-chrono --with-date_time --with-serialization install \
    && rm -rf /tmp/boost;
