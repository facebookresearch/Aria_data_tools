/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <sophus/se3.hpp>
#include <map>
#include <optional>
#include <string>

namespace ark {
namespace datatools {
namespace dataprovider {

void getDirContent(const std::string& dirPath, std::vector<std::string>& dirContent);

std::map<uint64_t, Sophus::SE3d> readPosesFromCsvFile(
    const std::string& inputPoseCsv,
    const int firstN = -1);

std::map<uint64_t, Eigen::Vector2f> readEyetrackingFromCsvFile(
    const std::string& inputEyetrackingCsv,
    const int firstN = -1);

std::vector<std::string> strSplit(const std::string& s, const char delimiter);

std::optional<Sophus::SE3d> queryPose(
    const uint64_t timestamp,
    const std::map<uint64_t, Sophus::SE3d>& timestampToPose);

std::optional<Eigen::Vector2f> queryEyetrack(
    const uint64_t timestamp,
    const std::map<uint64_t, Eigen::Vector2f>& timestampToEyetrack);

} // namespace dataprovider
} // namespace datatools
} // namespace ark
