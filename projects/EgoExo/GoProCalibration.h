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

#include <array>
#include <vector>

namespace ego_exo {

//
// GoPro Camera data
//

// GoPro intrinsic calib, and extrinsic pose in the world frame
struct GoProCalibration {
  // GoPro's unique identifier, currently we are using path of the video file
  std::string uid;
  // GoPro's pose in world frame
  Sophus::SE3d T_world_gopro;
  // image size
  int width, height;
  // cam intrinsic calibration params ("fx, fy, cx, cy, k1, k2, k3, k4")
  std::array<float, 8> intrinsics;
};

using GoProCalibrations = std::vector<GoProCalibration>;

} // namespace ego_exo
