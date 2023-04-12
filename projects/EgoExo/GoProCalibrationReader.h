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

#include "GoProCalibration.h"

#include <string>

namespace ego_exo {

constexpr std::array<const char*, 18> kGoProPoseHeader = {
    "gopro_uid",
    "tx_world_gopro",
    "ty_world_gopro",
    "tz_world_gopro",
    "qx_world_gopro",
    "qy_world_gopro",
    "qz_world_gopro",
    "qw_world_gopro",
    "image_width",
    "image_height",
    "intrinsics_0",
    "intrinsics_1",
    "intrinsics_2",
    "intrinsics_3",
    "intrinsics_4",
    "intrinsics_5",
    "intrinsics_6",
    "intrinsics_7",
};

GoProCalibrations loadGoProCalibrations(const std::string& fileName);

} // namespace ego_exo
