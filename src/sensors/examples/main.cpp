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

#include <models/DeviceModel.h>
#include <utility/VrsUtils.h>
#include <vrs/RecordFileReader.h>

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <Eigen/Eigen>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;
using namespace ark::datatools;

namespace {

std::string getCalibStrFromFile(const std::string& filePath) {
  auto ext = fs::path(filePath).extension();
  if (ext == ".json") {
    std::ifstream fin(filePath);
    if (!fin.is_open()) {
      std::cerr << fmt::format("Unable to find file: {}.", filePath) << std::endl;
      assert(false);
    }
    std::ostringstream sstr;
    sstr << fin.rdbuf();
    fin.close();
    return sstr.str();
  } else if (ext == ".vrs") {
    vrs::RecordFileReader reader;
    reader.openFile(filePath);
    return sensors::getCalibrationFromVrsFile(reader);
  } else {
    std::cerr << "Unsupported file type: " << ext << std::endl;
    assert(false);
  }
  // return {} to remove compiler warning
  return {};
}

} // namespace

int main(int argc, char** argv) {
  // Load calibration file.
  if (argc <= 1) {
    std::cerr << "Usage: sensors_example [<path_to_calib>.json | <path_to_vrs>.vrs]" << std::endl;
    return 1;
  }
  sensors::DeviceModel model = sensors::DeviceModel::fromJson(getCalibStrFromFile(argv[1]));

  std::cout << fmt::format("Successfully loaded calibration file from: {}.", argv[1]) << std::endl;

  Eigen::Vector3d p_slamLeft{3.0, 2.0, 1.0};
  std::cout << fmt::format("Original point in camera-slam-left frame: {}.", p_slamLeft.transpose())
            << std::endl;

  // Transform a 3D point from one frame to another.
  Eigen::Vector3d p_imuLeft = model.transform(p_slamLeft, "camera-slam-left", "imu-left");
  std::cout << fmt::format("Transformed to imu-left frame: {}.", p_imuLeft.transpose())
            << std::endl;

  // Project a 3D point to image plane and back.
  const auto slamLeft = model.getCameraCalib("camera-slam-left").value();
  Eigen::Vector2d uv_slamLeft = slamLeft.projectionModel.project(p_slamLeft);
  std::cout << fmt::format("Projected to image plane: {}.", uv_slamLeft.transpose()) << std::endl;
  Eigen::Vector3d p_slamLeft_convertBack = slamLeft.projectionModel.unproject(uv_slamLeft);
  std::cout << fmt::format("Unprojected to camera frame: {}.", p_slamLeft_convertBack.transpose())
            << std::endl;

  // Rectify a gyro 3D vector in IMU space.
  p_imuLeft = {0.1, 0.2, 0.3};
  std::cout << fmt::format("Original vector in imu-left frame: {}.", p_imuLeft.transpose())
            << std::endl;
  Eigen::Vector3d p_imuLeft_gyroRectified =
      model.getImuCalib("imu-left").value().gyro.rectify(p_imuLeft);
  std::cout << fmt::format("Rectified gyro vector: {}.", p_imuLeft_gyroRectified.transpose())
            << std::endl;

  return 0;
}
