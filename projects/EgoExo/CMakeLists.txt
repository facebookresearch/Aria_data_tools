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

message("--- Compiling EgoExo.")

cmake_minimum_required(VERSION 3.12)

project(AriaPilotDatasetTools
    VERSION 0.1
    DESCRIPTION "Utilities for the EgoExo dataset."
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build in Release mode by default
if (NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif (NOT CMAKE_BUILD_TYPE AND NOT MSVC)

# This folder is "Project" using aria data tools core libraries
add_subdirectory(../../src ariaDataToolsCore EXCLUDE_FROM_ALL)

find_package(Pangolin QUIET)

find_package(Boost REQUIRED COMPONENTS iostreams)

if (Pangolin_FOUND)

    add_library(compressed_istream CompressedIStream.cpp CompressedIStream.h)
    target_link_libraries(compressed_istream PUBLIC Boost::iostreams)

    add_library(data_loaders
        GlobalPointCloud.h
        GlobalPointCloudReader.h GlobalPointCloudReader.cpp
        GoProCalibration.h
        GoProCalibrationReader.h GoProCalibrationReader.cpp)
    target_link_libraries(data_loaders PUBLIC mps_io compressed_istream)
    target_include_directories(data_loaders
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            "../../src/third_party/"#fast-cpp-csv-parser
    )

    add_executable(egoExoViewer
        main.cpp
        Data3DGui.cpp Data3DGui.h
    )
    target_link_libraries(egoExoViewer
        PRIVATE
            data_loaders
            ${Pangolin_LIBRARIES}
    )
else()
    message("Pangolin has not been found. We can't compile this 'egoExoViewer' binary")
endif()
