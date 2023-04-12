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

#include "GlobalPointCloud.h"

#include "CompressedIStream.h"

#include <array>
#include <vector>

namespace ego_exo {

constexpr std::array<const char*, 8> kGlobalPointCloudColumns = {
    "uid",
    "px_world",
    "py_world",
    "pz_world",
    "num_observations",
    "inv_dist_std",
    "dist_std",
    "norm_image_gradient_squared"};

GlobalPointCloud readGlobalPointCloud(
    const std::string& path,
    const utils::StreamCompressionMode compression);

} // namespace ego_exo
