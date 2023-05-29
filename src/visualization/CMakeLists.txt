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

find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)

add_library(aria_viewer_lib AriaViewerBase.cpp AriaViewerBase.h)
target_link_libraries(aria_viewer_lib
        PUBLIC
        dataprovider
        Sophus::Sophus)
target_include_directories(aria_viewer_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

message("--- Compiling alpharobotics_homealone_kit_viewer.")
add_executable(alpharobotics_homealone_kit_viewer main.cpp AriaViewer.cpp)
target_link_libraries(alpharobotics_homealone_kit_viewer PRIVATE aria_viewer_lib mps_io ${Pangolin_LIBRARIES} Threads::Threads)

message("--- Compiling alpharobotics_homealone_kit_multi_viewer.")
add_executable(alpharobotics_homealone_kit_multi_viewer main_multi.cpp AriaViewer.cpp)
target_link_libraries(alpharobotics_homealone_kit_multi_viewer PRIVATE aria_viewer_lib ${Pangolin_LIBRARIES} Threads::Threads)

message("--- Compiling alpharobotics_homealone_kit_sensor_viewer.")
add_executable(alpharobotics_homealone_kit_sensor_viewer main_aria_sensors_viewer.cpp)
target_link_libraries(alpharobotics_homealone_kit_sensor_viewer PRIVATE dataprovider ${Pangolin_LIBRARIES})