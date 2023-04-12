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

#include "GoProCalibrationReader.h"

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include <fast-cpp-csv-parser/csv.h>

#include <iostream>

namespace ego_exo {

GoProCalibrations loadGoProCalibrations(const std::string& fileName) {
  GoProCalibrations poses;

  io::CSVReader<kGoProPoseHeader.size()> csv(fileName);
  // Read in the CSV header
  const auto readHeader = [&](auto&&... args) { csv.read_header(io::ignore_no_column, args...); };
  std::apply(readHeader, kGoProPoseHeader);

  std::string gopro_uid;
  Eigen::Vector3d t_world_gopro;
  Eigen::Quaterniond q_world_gopro;
  int width, height;
  std::array<float, 8> intrinsics;

  while (csv.read_row(
      gopro_uid,
      t_world_gopro.x(),
      t_world_gopro.y(),
      t_world_gopro.z(),
      q_world_gopro.x(),
      q_world_gopro.y(),
      q_world_gopro.z(),
      q_world_gopro.w(),
      width,
      height,
      intrinsics[0],
      intrinsics[1],
      intrinsics[2],
      intrinsics[3],
      intrinsics[4],
      intrinsics[5],
      intrinsics[6],
      intrinsics[7])) {
    auto& pose = poses.emplace_back();
    pose.uid = gopro_uid;
    pose.T_world_gopro = Sophus::SE3d(q_world_gopro, t_world_gopro);
    pose.width = width;
    pose.height = height;
    pose.intrinsics = intrinsics;
  }
  std::cout << "Loaded #GoProCalibration data: " << poses.size() << std::endl;

  return poses;
}

} // namespace ego_exo
