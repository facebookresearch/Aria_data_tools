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

#include <camera/projection/FisheyeRadTanThinPrism.h>
#include <camera/projection/KannalaBrandtK3.h>

#include <cereal/external/rapidjson/rapidjson.h>

namespace ark {
namespace datatools {
namespace sensors {

namespace {

Eigen::VectorXd parseVectorXdFromJson(const fb_rapidjson::Value& json) {
  Eigen::VectorXd vec(json.Size());
  for (size_t i = 0; i < json.Size(); ++i) {
    vec(i) = json[i].GetDouble();
  }
  return vec;
}

Eigen::Vector3d parseVector3dFromJson(const fb_rapidjson::Value& json) {
  assert(json.Size() == 3);

  return {json[0].GetDouble(), json[1].GetDouble(), json[2].GetDouble()};
}

Eigen::Matrix3d parseMatrix3dFromJson(const fb_rapidjson::Value& json) {
  assert(json.Size() == 3);

  Eigen::Matrix3d mat;
  for (size_t i = 0; i < 3; ++i) {
    mat.row(i) = parseVector3dFromJson(json[i]).transpose().eval();
  }
  return mat;
}

Sophus::SE3d parseSe3dFromJson(const fb_rapidjson::Value& json) {
  Eigen::Vector3d translation = parseVector3dFromJson(json["Translation"]);

  assert(json["UnitQuaternion"].Size() == 2);
  double qReal = json["UnitQuaternion"][0].GetDouble();
  Eigen::Vector3d qImag = parseVector3dFromJson(json["UnitQuaternion"][1]);

  Eigen::Quaterniond rotation(qReal, qImag.x(), qImag.y(), qImag.z());
  return {rotation, translation};
}

CameraCalibration parseCameraCalibFromJson(const fb_rapidjson::Value& json) {
  CameraCalibration camCalib;
  camCalib.label = json["Label"].GetString();
  camCalib.T_Device_Camera = parseSe3dFromJson(json["T_Device_Camera"]);

  const std::string projectionModelName = json["Projection"]["Name"].GetString();
  if (projectionModelName == "FisheyeRadTanThinPrism") {
    camCalib.projectionModel.modelName = CameraProjectionModel::ModelType::Fisheye624;
  } else if (projectionModelName == "KannalaBrandtK3") {
    camCalib.projectionModel.modelName = CameraProjectionModel::ModelType::KannalaBrandtK3;
  }
  camCalib.projectionModel.projectionParams = parseVectorXdFromJson(json["Projection"]["Params"]);

  return camCalib;
}

LinearRectificationModel parseRectModelFromJson(const fb_rapidjson::Value& json) {
  LinearRectificationModel model;
  model.rectificationMatrix = parseMatrix3dFromJson(json["Model"]["RectificationMatrix"]);
  model.bias = parseVector3dFromJson(json["Bias"]["Offset"]);
  return model;
}

ImuCalibration parseImuCalibFromJson(const fb_rapidjson::Value& json) {
  ImuCalibration imuCalib;
  imuCalib.label = json["Label"].GetString();
  imuCalib.accel = parseRectModelFromJson(json["Accelerometer"]);
  imuCalib.gyro = parseRectModelFromJson(json["Gyroscope"]);
  imuCalib.T_Device_Imu = parseSe3dFromJson(json["T_Device_Imu"]);
  return imuCalib;
}

} // namespace

Eigen::Vector2d CameraProjectionModel::project(const Eigen::Vector3d& p) const {
  switch (modelName) {
    case ModelType::KannalaBrandtK3:
      return KannalaBrandtK3Projection::project(p, projectionParams);
    case ModelType::Fisheye624:
      return Fisheye624::project(p, projectionParams);
      // Intentionally skipping default to raise a compile error when new models are added.
  }
  assert(false);
}

Eigen::Vector3d CameraProjectionModel::unproject(const Eigen::Vector2d& uv) const {
  switch (modelName) {
    case ModelType::KannalaBrandtK3:
      return KannalaBrandtK3Projection::unproject(uv, projectionParams);
    case ModelType::Fisheye624:
      return Fisheye624::unproject(uv, projectionParams);
      // Intentionally skipping default to raise a compile error when new models are added.
  }
  assert(false);
}

Eigen::Vector3d LinearRectificationModel::rectify(const Eigen::Vector3d& v_raw) const {
  return rectificationMatrix.inverse() * (v_raw - bias);
}

std::optional<CameraCalibration> DeviceModel::getCameraCalib(const std::string& label) const {
  if (cameraCalibs_.find(label) == cameraCalibs_.end()) {
    return {};
  }
  return cameraCalibs_.at(label);
}

std::optional<ImuCalibration> DeviceModel::getImuCalib(const std::string& label) const {
  if (imuCalibs_.find(label) == imuCalibs_.end()) {
    return {};
  }
  return imuCalibs_.at(label);
}

DeviceModel DeviceModel::fromJson(const fb_rapidjson::Document& json) {
  DeviceModel calib;
  if (json.FindMember("CameraCalibrations") != json.MemberEnd()) {
    for (const auto& camJson : json["CameraCalibrations"].GetArray()) {
      CameraCalibration camCalib = parseCameraCalibFromJson(camJson);
      calib.cameraCalibs_[camCalib.label] = camCalib;
    }
  }
  if (json.FindMember("ImuCalibrations") != json.MemberEnd()) {
    for (const auto& imuJson : json["ImuCalibrations"].GetArray()) {
      ImuCalibration imuCalib = parseImuCalibFromJson(imuJson);
      calib.imuCalibs_[imuCalib.label] = imuCalib;
    }
  }
  return calib;
}

DeviceModel DeviceModel::fromJson(const std::string& jsonStr) {
  fb_rapidjson::Document doc;
  doc.Parse(jsonStr.c_str());
  return DeviceModel::fromJson(doc);
}

Eigen::Vector3d DeviceModel::transform(
    const Eigen::Vector3d& p_Source,
    const std::string& sourceSensorLabel,
    const std::string& destSensorLabel) const {
  Sophus::SE3d T_Device_Source, T_Device_Dest;
  if (cameraCalibs_.find(sourceSensorLabel) != cameraCalibs_.end()) {
    T_Device_Source = cameraCalibs_.at(sourceSensorLabel).T_Device_Camera;
  } else if (imuCalibs_.find(sourceSensorLabel) != imuCalibs_.end()) {
    T_Device_Source = imuCalibs_.at(sourceSensorLabel).T_Device_Imu;
  } else {
    assert(false);
  }

  if (cameraCalibs_.find(destSensorLabel) != cameraCalibs_.end()) {
    T_Device_Dest = cameraCalibs_.at(destSensorLabel).T_Device_Camera;
  } else if (imuCalibs_.find(destSensorLabel) != imuCalibs_.end()) {
    T_Device_Dest = imuCalibs_.at(destSensorLabel).T_Device_Imu;
  } else {
    assert(false);
  }

  return T_Device_Dest.inverse() * T_Device_Source * p_Source;
}

std::vector<std::string> DeviceModel::getCameraLabels() const {
  std::vector<std::string> cameraLabels;
  for (const auto& [key, _] : cameraCalibs_) {
    cameraLabels.push_back(key);
  }
  return cameraLabels;
}

std::vector<std::string> DeviceModel::getImuLabels() const {
  std::vector<std::string> imuLabels;
  for (const auto& [key, _] : imuCalibs_) {
    imuLabels.push_back(key);
  }
  return imuLabels;
}

} // namespace sensors
} // namespace datatools
} // namespace ark