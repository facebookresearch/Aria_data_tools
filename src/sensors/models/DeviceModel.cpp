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

MagnetometerCalibration parseMagnetometerCalibrationFromJson(const fb_rapidjson::Value& json) {
  MagnetometerCalibration magnetometerCalibration;
  magnetometerCalibration.label = json["Label"].GetString();
  magnetometerCalibration.model = parseRectModelFromJson(json);
  return magnetometerCalibration;
}

BarometerCalibration parseBarometerCalibrationFromJson(const fb_rapidjson::Value& json) {
  BarometerCalibration barometerCalibration;
  barometerCalibration.label = json["Label"].GetString();
  barometerCalibration.pressure.slope = json["PressureModel"]["Slope"].GetDouble();
  barometerCalibration.pressure.offsetPa = json["PressureModel"]["OffsetPa"].GetDouble();
  return barometerCalibration;
}

MicrophoneCalibration parseMicrophoneCalibrationFromJson(const fb_rapidjson::Value& json) {
  MicrophoneCalibration microphoneCalibration;
  microphoneCalibration.label = json["Label"].GetString();
  microphoneCalibration.dSensitivity1KDbv = json["DSensitivity1KDbv"].GetDouble();
  return microphoneCalibration;
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

std::optional<MagnetometerCalibration> DeviceModel::getMagnetometerCalib(
    const std::string& label) const {
  if (magnetometerCalibs_.find(label) == magnetometerCalibs_.end()) {
    return {};
  }
  return magnetometerCalibs_.at(label);
}

std::optional<BarometerCalibration> DeviceModel::getBarometerCalib(const std::string& label) const {
  if (barometerCalibs_.find(label) == barometerCalibs_.end()) {
    return {};
  }
  return barometerCalibs_.at(label);
}

std::optional<MicrophoneCalibration> DeviceModel::getMicrophoneCalib(
    const std::string& label) const {
  if (microphoneCalibs_.find(label) == microphoneCalibs_.end()) {
    return {};
  }
  return microphoneCalibs_.at(label);
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
  if (json.FindMember("BaroCalibrations") != json.MemberEnd()) {
    for (const auto& barometerJson : json["BaroCalibrations"].GetArray()) {
      BarometerCalibration barometerCalib = parseBarometerCalibrationFromJson(barometerJson);
      calib.barometerCalibs_[barometerCalib.label] = barometerCalib;
    }
  }
  if (json.FindMember("MagCalibrations") != json.MemberEnd()) {
    for (const auto& magnetometerJson : json["MagCalibrations"].GetArray()) {
      MagnetometerCalibration magnetometerCalib =
          parseMagnetometerCalibrationFromJson(magnetometerJson);
      calib.magnetometerCalibs_[magnetometerCalib.label] = magnetometerCalib;
    }
  }
  if (json.FindMember("MicCalibrations") != json.MemberEnd()) {
    for (const auto& microphoneJson : json["MicCalibrations"].GetArray()) {
      MicrophoneCalibration microphoneCalib = parseMicrophoneCalibrationFromJson(microphoneJson);
      calib.microphoneCalibs_[microphoneCalib.label] = microphoneCalib;
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

std::vector<std::string> DeviceModel::getMagnetometerLabels() const {
  std::vector<std::string> magnetometerLabels;
  for (const auto& [key, _] : magnetometerCalibs_) {
    magnetometerLabels.push_back(key);
  }
  return magnetometerLabels;
}

std::vector<std::string> DeviceModel::getBarometerLabels() const {
  std::vector<std::string> barometerLabels;
  for (const auto& [key, _] : barometerCalibs_) {
    barometerLabels.push_back(key);
  }
  return barometerLabels;
}

std::vector<std::string> DeviceModel::getMicrophoneLabels() const {
  std::vector<std::string> microphoneLabels;
  for (const auto& [key, _] : microphoneCalibs_) {
    microphoneLabels.push_back(key);
  }
  return microphoneLabels;
}

namespace {
const int kFocalIdx = 0;
const int kPrincipalPointColIdx = 1;
const int kPrincipalPointRowIdx = 2;
} // namespace

bool DeviceModel::tryCropAndScaleCameraCalibration(
    const std::string& label,
    const int nativeResolution,
    const int newWidth) {
  // Camera calibration should be rectified only once
  if (updatedCameraCalibs_.count(label)) {
    return true;
  }
  if (cameraCalibs_.find(label) != cameraCalibs_.end()) {
    auto& cameraCalib = cameraCalibs_.at(label);
    // Aria supports two RGB resolution:
    // - Full res 2880x2880 & Medium res 1408x1408
    // We are applying here the necessary intrinsics parameters change
    //  to fit camera calibration to the used resolution
    if (label == "camera-rgb") {
      Eigen::VectorXd& camParams = cameraCalib.projectionModel.projectionParams;
      // Testing if principal point is appropriate for this image width
      if (camParams[kPrincipalPointColIdx] * 2 >
          newWidth) { // We need to rescale calibration parameters

        // Assume the resolution change follows the following steps:
        // - centered cropping -> sensor pixel binning
        const double rescaleFactor = std::floor(
            nativeResolution / static_cast<double>(newWidth)); // binning can only be an integer
        const double halfCroppedSize = (nativeResolution - newWidth * rescaleFactor) / 2.0;
        camParams[kPrincipalPointColIdx] -= halfCroppedSize;
        camParams[kPrincipalPointRowIdx] -= halfCroppedSize;

        camParams[kFocalIdx] /= rescaleFactor;
        camParams[kPrincipalPointColIdx] /= rescaleFactor;
        camParams[kPrincipalPointRowIdx] /= rescaleFactor;

        updatedCameraCalibs_.insert(label);
        return true;
      }
    }
  }
  return false;
}

} // namespace sensors
} // namespace datatools
} // namespace ark
