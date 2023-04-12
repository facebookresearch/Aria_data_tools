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

#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "Data3DGui.h"
#include "GlobalPointCloudReader.h"
#include "GoProCalibrationReader.h"
#include "trajectoryReader.h"

using namespace ark::datatools;

namespace {

// Return the list of files included in a given folder
std::vector<std::string> getDirContent(const std::filesystem::path& dirPath) {
  std::set<std::string> pathSet;
  for (auto& p : std::filesystem::directory_iterator(dirPath)) {
    pathSet.insert(p.path());
  }
  return {pathSet.begin(), pathSet.end()};
}

std::vector<std::string> findMatchingFilenames(
    const std::vector<std::string>& dirContent,
    const std::string& pattern) {
  std::vector<std::string> matchingFilenames;
  std::for_each(
      dirContent.begin(),
      dirContent.end(),
      [&matchingFilenames, pattern](const std::string& filename) {
        if (filename.find(pattern) != std::string::npos) {
          matchingFilenames.push_back(filename);
        }
      });

  std::cout << "Found " << pattern << " data: " << std::endl;
  std::copy(
      matchingFilenames.begin(),
      matchingFilenames.end(),
      std::ostream_iterator<std::string>(std::cout, ",\n"));
  return matchingFilenames;
}

} // namespace

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    std::cout << "Please specify the folder where to read EgoExo data." << std::endl;
    return EXIT_FAILURE;
  }

  const std::string entryPath = argv[1];

  // Get the content of the folder
  const std::vector<std::string> dirContent = getDirContent(entryPath);

  //
  // Find & Read trajectory data
  //
  const std::vector<std::string> trajectoryFilenames =
      findMatchingFilenames(dirContent, "closed_loop_trajectory");

  // Load each trajectories
  std::vector<std::vector<Eigen::Vector3f>> fullTrajs_world;
  fullTrajs_world.reserve(trajectoryFilenames.size());
  for (const auto& it : trajectoryFilenames) {
    Trajectory loadedPoses = readCloseLoop(it);
    std::vector<Eigen::Vector3f> points;
    points.reserve(loadedPoses.size());
    for (const auto& pose : loadedPoses) {
      points.push_back(pose.T_world_device.translation().cast<float>());
    }
    fullTrajs_world.push_back(std::move(points));
  }

  //
  // Find & Read point cloud data
  //

  const std::vector<std::string> pointCloudFilenames =
      findMatchingFilenames(dirContent, "global_points");

  // Load point cloud (can be .csv or .gz)
  const ego_exo::GlobalPointCloud ptCloud = pointCloudFilenames.empty()
      ? ego_exo::GlobalPointCloud()
      : ego_exo::readGlobalPointCloud(
            pointCloudFilenames[0],
            std::filesystem::path(pointCloudFilenames[0]).extension() == ".csv"
                ? utils::StreamCompressionMode::NONE
                : utils::StreamCompressionMode::GZIP);

  //
  // Find & Read GoPro calibration data
  //
  const std::vector<std::string> goProFilenames =
      findMatchingFilenames(dirContent, "gopro_calibrations");

  const auto goproCalibs = goProFilenames.empty()
      ? std::vector<ego_exo::GoProCalibration>()
      : ego_exo::loadGoProCalibrations(goProFilenames[0]);

  // GUI
  ego_exo::Data3DGui gui3d(ptCloud, fullTrajs_world, goproCalibs);

  while (!pangolin::ShouldQuit()) {
    // draw without raw image and current rig
    gui3d.draw();

    pangolin::FinishFrame();
  }
  return 0;
}
