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

#include <cereal/external/rapidjson/document.h>
#include <sophus/se3.hpp>
#include <Eigen/Core>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ark {
namespace datatools {
namespace sensors {

struct CameraProjectionModel {
  enum class ModelType {
    // Kannala and Brandt Like 'Generic' Projection Model
    // http://cs.iupui.edu/~tuceryan/pdf-repository/Kannala2006.pdf
    // https://april.eecs.umich.edu/wiki/Camera_suite
    KannalaBrandtK3,
    Fisheye624,
  };
  ModelType modelName;
  Eigen::VectorXd projectionParams;

  Eigen::Vector2d project(const Eigen::Vector3d& p) const;
  Eigen::Vector3d unproject(const Eigen::Vector2d& uv) const;
};

struct CameraCalibration {
  std::string label;
  CameraProjectionModel projectionModel;
  Sophus::SE3d T_Device_Camera;
};

struct LinearRectificationModel {
  Eigen::Matrix3d rectificationMatrix;
  Eigen::Vector3d bias;

  // Compensates the input vector (acceleration for accelerator, or angular
  // velocity for gyroscope) with a linear model:
  // v_compensated = A.inv() * (v_raw - b)
  Eigen::Vector3d rectify(const Eigen::Vector3d& v_raw) const;
};

struct ImuCalibration {
  std::string label;
  LinearRectificationModel accel;
  LinearRectificationModel gyro;
  Sophus::SE3d T_Device_Imu;
};

class DeviceModel {
 public:
  static DeviceModel fromJson(const fb_rapidjson::Document& json);
  static DeviceModel fromJson(const std::string& jsonStr);

  std::optional<CameraCalibration> getCameraCalib(const std::string& label) const;
  std::optional<ImuCalibration> getImuCalib(const std::string& label) const;

  std::vector<std::string> getCameraLabels() const;
  std::vector<std::string> getImuLabels() const;

  Eigen::Vector3d transform(
      const Eigen::Vector3d& p_source,
      const std::string& sourceSensorLabel,
      const std::string& destSensorLabel) const;

 private:
  std::map<std::string, CameraCalibration> cameraCalibs_;
  std::map<std::string, ImuCalibration> imuCalibs_;
};

} // namespace sensors
} // namespace datatools
} // namespace ark
