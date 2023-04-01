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

message("--- Compiling dataset_provider Pilot dataset.")

find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)

add_library(pilotDatasetProvider_lib
    ./pilot/PilotDataset.cpp
    ./pilot/PilotDataset.h
    ./pilot/speech_to_text_datum.h)
target_link_libraries(pilotDatasetProvider_lib
    PUBLIC
        vrsdataprovider
        Sophus::Sophus
        )
target_include_directories(pilotDatasetProvider_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        "../third_party/include/fast-cpp-csv-parser")

add_executable(pilotDataset_viewer
    ./pilot/viewer/main.cpp
    ./pilot/viewer/PilotDatasetViewer.cpp
    ./pilot/viewer/PilotDatasetViewer.h)
target_link_libraries(pilotDataset_viewer
    PRIVATE
    pilotDatasetProvider_lib
    aria_viewer_lib
    ${Pangolin_LIBRARIES}
    Threads::Threads)
