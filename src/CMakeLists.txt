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

cmake_minimum_required(VERSION 3.12)

project(ark
    VERSION 0.1
    DESCRIPTION "SDK and utilities for Project Aria datasets."
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build in Release mode by default
if (NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif (NOT CMAKE_BUILD_TYPE AND NOT MSVC)

# If available, use ccache
include(${CMAKE_MODULE_PATH}/CCache.cmake)

find_package(vrslib REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Sophus REQUIRED)

option(BUILD_PYARK "Build python binding." OFF)
option(BUILD_ARK_TESTS "Build tests." ON)
option(BUILD_ARK_EXAMPLES "Build examples." ON)

if(BUILD_ARK_TESTS)
    enable_testing()
endif(BUILD_ARK_TESTS)

if(BUILD_PYARK)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG master
    )
    FetchContent_MakeAvailable(pybind11)

    add_subdirectory(${pybind11_SOURCE_DIR})
    pybind11_add_module(pyark ${CMAKE_CURRENT_SOURCE_DIR}/python/bindings.cpp)
    target_link_libraries(pyark PUBLIC aria_viewer_lib dataprovider sensors vrs::vrslib)
endif(BUILD_PYARK)


add_subdirectory(data_provider)
add_subdirectory(sensors)
add_subdirectory(visualization)
