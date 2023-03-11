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

# Ensure a SUDO command is available in the container
RUN if type sudo 2>/dev/null; then \
     echo "The sudo command already exists... Skipping."; \
    else \
     echo "#!/bin/sh\n\${@}" > /usr/sbin/sudo; \
     chmod +x /usr/sbin/sudo; \
    fi

# Get dependencies
RUN apt-get update --fix-missing && DEBIAN_FRONTEND="noninteractive" TZ="America/New_York" apt-get install -y tzdata --fix-missing; sudo apt upgrade -y --fix-missing

# Code
ADD ./ /opt/aria_data_tools

# Install build
RUN cd /opt/aria_data_tools/src/scripts/; sh ./install_deps_debian.sh

# Configure
RUN mkdir /opt/aria_data_tools_Build; cd /opt/aria_data_tools_Build; cmake -DCMAKE_BUILD_TYPE=RELEASE /opt/aria_data_tools/src;

# Build & test
RUN cd /opt/aria_data_tools_Build; make -j ; ctest -j;

# Build python bindings
RUN cd /opt/aria_data_tools/src; pip3 install --global-option=build_ext --global-option="-j8" .;

# Link shared Pangolin libraries
RUN ldconfig;
