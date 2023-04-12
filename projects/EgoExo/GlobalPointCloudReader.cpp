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

#include "GlobalPointCloudReader.h"

#include "CompressedIStream.h"

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include <fast-cpp-csv-parser/csv.h>

#include <iostream>

namespace ego_exo {

GlobalPointCloud readGlobalPointCloud(
    const std::string& path,
    const utils::StreamCompressionMode compression) {
  utils::CompressedIStream istream(path, compression);
  io::CSVReader<kGlobalPointCloudColumns.size()> csv(path.c_str(), istream);

  // Read in the CSV header
  const auto readHeader = [&](auto&&... args) { csv.read_header(io::ignore_no_column, args...); };
  std::apply(readHeader, kGlobalPointCloudColumns);

  GlobalPointCloud cloud;
  GlobalPointPosition point;

  while (csv.read_row(
      point.uid,
      point.position_world.x(),
      point.position_world.y(),
      point.position_world.z(),
      point.numObservations,
      point.inverseDistanceStd,
      point.distanceStd,
      point.normImageGradientSquared)) {
    cloud.push_back(point);
  }

  std::cout << "Loaded #3dPoints: " << cloud.size() << std::endl;

  return cloud;
}

} // namespace ego_exo
