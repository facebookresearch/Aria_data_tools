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

export CIBW_ARCHS_MACOS=arm64
export CIBW_BUILD="cp36-*64 cp37-*64 cp38-*64 cp39-*64 cp310-*64 cp311-*64"
export CIBW_BEFORE_BUILD_MACOS="arch -arm64 brew install boost cereal cmake fmt glog jpeg-turbo libpng lz4 xxhash zstd"
export CIBW_SKIP="*-manylinux_i686 *musllinux*"

# This will set -DCMAKE_OSX_ARCHITECTURES="arm64" in setup.py
export ARCHFLAGS="-arch arm64"

export CMAKE_BUILD_PARALLEL_LEVEL=8
# export CMAKE_ARGS="-DCMAKE_OSX_DEPLOYMENT_TARGET=13.0",
export CMAKE_ARGS="-DCMAKE_PREFIX_PATH=/Users/${USER}/homebrew"

# Use clang and not gcc
export CC=/usr/bin/cc
export CXX=/usr/bin/c++
pip3 install cibuildwheel
python3 -m cibuildwheel --output-dir dist --platform macos

export CIBW_ARCHS_MACOS=
export CIBW_BUILD=
export CIBW_BEFORE_BUILD_MACOS=
export CIBW_SKIP=
export ARCHFLAGS=
export CMAKE_BUILD_PARALLEL_LEVEL=
export CMAKE_ARGS=
export CC=
export CXX=
