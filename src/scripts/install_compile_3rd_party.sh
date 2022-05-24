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

if [[ "$OSTYPE" == "darwin"* ]]; then
    thread=$(gnproc)
else
    thread=$(nproc)
fi

# Build Fmt
cd /tmp; git clone https://github.com/fmtlib/fmt.git -b 8.1.1 \
    && cd fmt \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE -DFMT_TEST=OFF .; sudo make -j$thread install; rm -rf /tmp/fmt;

# Build Sophus
cd /tmp; git clone https://github.com/strasdat/Sophus.git -b v22.04.1\
    && cd Sophus \
    && cmake -DCMAKE_BUILD_TYPE=Release  -DBUILD_SOPHUS_TESTS=OFF .; sudo make -j$thread install; rm -rf /tmp/Sophus;

# Build Cereal
cd /tmp; git clone https://github.com/USCiLab/cereal.git -b v1.3.2\
    && cd cereal \
    && cmake -DSKIP_PORTABILITY_TEST=1 -DJUST_INSTALL_CEREAL=ON .; sudo make -j$thread install; rm -rf /tmp/cereal;

# Build Pangolin
cd /tmp; git clone https://github.com/stevenlovegrove/Pangolin.git -b v0.8 --recursive \
    && mkdir Pangolin_Build && cd Pangolin_Build \
    && cmake -DCMAKE_BUILD_TYPE=Release  ../Pangolin/ . \
    && sudo make -j$thread install; sudo cmake --build . -t pypangolin_pip_install;

# Build VRS
cd /tmp; git clone https://github.com/facebookresearch/vrs.git \
    && mkdir vrs_Build && cd vrs_Build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE ../vrs/ .; sudo make -j$thread install; rm -rf /tmp/vrs /tmp/vrs_Build;
