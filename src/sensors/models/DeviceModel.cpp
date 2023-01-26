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

#include <functional>
#include <iostream>

#include <cereal/external/rapidjson/document.h>
#include <models/DeviceModel.h>

#include <camera/projection/FisheyeRadTanThinPrism.h>
#include <camera/projection/KannalaBrandtK3.h>

#include <cereal/external/rapidjson/rapidjson.h>

namespace ark {
namespace datatools {
namespace sensors {

namespace {

// Circular mask radius value for full resolution RGB and SLAM cameras
const int kSlamValidRadius = 330;
const int kRgbValidRadius = 1415;

// Hardcoded values to be used by getSensorPoseByLabel to compute poses
// for barometer, magnetometer and microphone sensors
// There are two subtypes for Aria devices: DVT-L(large) and DVT-S(small)
// Hardcoded values differ for the two subtypes
const std::unordered_map<std::string, const std::unordered_map<std::string, Sophus::SE3d>>
    T_Device_FrameMap = {
        {"DVT-L",
         {{"camera-slam-left",
           {Eigen::Quaterniond(
                0.6634014621838321,
                -0.20968650730402055,
                0.24474170230353254,
                -0.6753010941650868),
            {0.071351, 0.002372, 0.008454}}},
          {"baro0",
           {Eigen::Quaterniond(
                0.7044160331158363,
                -0.704416033115836,
                0.06162833998534202,
                -0.06162833998534204),
            {-0.009258, 0.010842, 0.0172}}},
          {"mag0",
           {Eigen::Quaterniond(
                -0.3354557015967031,
                0.6348637728730656,
                0.6250835690288524,
                -0.3060849455458301),
            {0.066201, -0.00576, -0.001777}}},
          {"mic0",
           {Eigen::Quaterniond(
                -0.4466732905655805,
                -0.462594953221541,
                0.5961358393213673,
                -0.48073999399455747),
            {-0.046137509961, -0.029296904188, 0.006233332458}}},
          {"mic1",
           {Eigen::Quaterniond(
                0.45451953667583,
                -0.5416751709096407,
                0.4545195366758299,
                0.541675170909641),
            {0.009200472201, 0.010304189016, 0.01725}}},
          {"mic2",
           {Eigen::Quaterniond(
                0.48073999399455714,
                0.5961358393213673,
                -0.4625949532215406,
                0.44667329056558025),
            {0.046137509961, -0.029296904188, 0.006233332458}}},
          {"mic3",
           {Eigen::Quaterniond(
                0.03700718883556693,
                0.024677753697836938,
                0.706676013509939,
                0.7061377262097094),
            {0.065924850784, 0.011961037562, 0.004304525213}}},
          {"mic4",
           {Eigen::Quaterniond(
                0.5533876349070789,
                -0.5684125695232376,
                0.4206033310715556,
                0.4401841821686134),
            {-0.054799747025, 0.013141924571, 0.01096}}},
          {"mic5",
           {Eigen::Quaterniond(
                0.0321917707643714,
                0.7543767868913739,
                0.029592009455780385,
                0.6549837145081323),
            {0.072318361733, 0.008272042771, -0.094954730047}}},
          {"mic6",
           {Eigen::Quaterniond(
                0.0321917707643717,
                0.7543767868913734,
                -0.02959200945578069,
                -0.6549837145081314),
            {-0.072318558345, 0.008271400968, -0.094954767162}}}}},
        {"DVT-S",
         {{"camera-slam-left",
           {Eigen::Quaterniond(
                0.6634014621838321,
                -0.20968650730402055,
                0.24474170230353254,
                -0.6753010941650868),
            {0.069051, 0.002372, 0.009254}}},
          {"baro0",
           {Eigen::Quaterniond(
                0.7044160331158363,
                -0.704416033115836,
                0.06162833998534202,
                -0.06162833998534204),
            {-0.009258, 0.010842, 0.0172}}},
          {"mag0",
           {Eigen::Quaterniond(
                -0.33199561459691707,
                0.641846614077474,
                0.6179113269940647,
                -0.30983451689269853),
            {0.064372, -0.005868, 0.000699}}},
          {"mic0",
           {Eigen::Quaterniond(
                0.46713221110590525,
                0.6064273312736143,
                0.4486845987290905,
                -0.4612109279873855),
            {-0.045906351881, -0.027938466628, 0.006667127264}}},
          {"mic1",
           {Eigen::Quaterniond(
                0.45451953667583,
                -0.5416751709096407,
                0.4545195366758299,
                0.541675170909641),
            {0.00916053312, 0.010230694799, 0.01725}}},
          {"mic2",
           {Eigen::Quaterniond(
                0.46713206676748886,
                0.6064275298986738,
                -0.4486854732162549,
                0.46120996227665667),
            {0.045905335057, -0.027931354925, 0.006668129433}}},
          {"mic3",
           {Eigen::Quaterniond(
                0.03700718883556693,
                0.024677753697836938,
                0.706676013509939,
                0.7061377262097094),
            {0.063470940948, 0.012033935758, 0.005565985947}}},
          {"mic4",
           {Eigen::Quaterniond(
                -0.4401841821686135,
                0.4206033310715548,
                0.5684125695232379,
                0.5533876349070787),
            {-0.052398255025, 0.013200030459, 0.01216}}},
          {"mic5",
           {Eigen::Quaterniond(
                0.03219180881231075,
                0.7543002196214468,
                0.029591714532785928,
                0.6550719018210435),
            {0.06985623699, 0.008269731192, -0.093105155761}}},
          {"mic6",
           {Eigen::Quaterniond(
                0.03219152449164331,
                0.7544533536981446,
                -0.029591740767486312,
                -0.6548955426042204),
            {-0.069821880534, 0.008268307286, -0.093137768576}}}}}};

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

  // Handle sensor valid radius
  if (camCalib.label == "camera-rgb") {
    camCalib.validRadius = kRgbValidRadius;
  } else if (camCalib.label == "camera-slam-left" || camCalib.label == "camera-slam-right") {
    camCalib.validRadius = kSlamValidRadius;
  }
  // else nothing is needed (value should be -1)

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

Sophus::SE3d getSensorPoseByLabel(
    const DeviceModel& calib,
    const std::string& label,
    const std::string& deviceSubtype) {
  if (!calib.getCameraCalib("camera-slam-left").has_value()) {
    std::cerr << "Camera calibration must exist for loading device model, exiting" << std::endl;
    exit(1);
  }
  return calib.getCameraCalib("camera-slam-left").value().T_Device_Camera *
      T_Device_FrameMap.at(deviceSubtype).at("camera-slam-left").inverse() *
      T_Device_FrameMap.at(deviceSubtype).at(label);
}

MagnetometerCalibration parseMagnetometerCalibrationFromJson(
    const fb_rapidjson::Value& json,
    const DeviceModel& calib,
    const std::string& deviceSubtype) {
  MagnetometerCalibration magnetometerCalibration;
  magnetometerCalibration.label = json["Label"].GetString();
  magnetometerCalibration.model = parseRectModelFromJson(json);
  magnetometerCalibration.T_Device_Magnetometer =
      getSensorPoseByLabel(calib, magnetometerCalibration.label, deviceSubtype);
  return magnetometerCalibration;
}

BarometerCalibration parseBarometerCalibrationFromJson(
    const fb_rapidjson::Value& json,
    const DeviceModel& calib,
    const std::string& deviceSubtype) {
  BarometerCalibration barometerCalibration;
  barometerCalibration.label = json["Label"].GetString();
  barometerCalibration.pressure.slope = json["PressureModel"]["Slope"].GetDouble();
  barometerCalibration.pressure.offsetPa = json["PressureModel"]["OffsetPa"].GetDouble();
  barometerCalibration.T_Device_Barometer =
      getSensorPoseByLabel(calib, barometerCalibration.label, deviceSubtype);
  return barometerCalibration;
}

MicrophoneCalibration parseMicrophoneCalibrationFromJson(
    const fb_rapidjson::Value& json,
    const DeviceModel& calib,
    const std::string& deviceSubtype) {
  MicrophoneCalibration microphoneCalibration;
  microphoneCalibration.label = json["Label"].GetString();
  microphoneCalibration.dSensitivity1KDbv = json["DSensitivity1KDbv"].GetDouble();
  microphoneCalibration.T_Device_Microphone =
      getSensorPoseByLabel(calib, microphoneCalibration.label, deviceSubtype);
  return microphoneCalibration;
}

} // namespace

Eigen::Vector2d CameraProjectionModel::getFocalLengths() const {
  switch (modelName) {
    case ModelType::KannalaBrandtK3:
      return {
          projectionParams[KannalaBrandtK3Projection::kFocalXIdx],
          projectionParams[KannalaBrandtK3Projection::kFocalYIdx]};
    case ModelType::Fisheye624:
      return {projectionParams[Fisheye624::kFocalXIdx], projectionParams[Fisheye624::kFocalYIdx]};
  }
  // Intentionally skipping default to raise a compile error when new models are added.
  assert(false);
  // return 0s to remove compiler warning
  return Eigen::Vector2d::Zero();
}

Eigen::Vector2d CameraProjectionModel::getPrincipalPoint() const {
  switch (modelName) {
    case ModelType::KannalaBrandtK3:
      return {
          projectionParams[KannalaBrandtK3Projection::kPrincipalPointColIdx],
          projectionParams[KannalaBrandtK3Projection::kPrincipalPointRowIdx]};
    case ModelType::Fisheye624:
      return {
          projectionParams[Fisheye624::kPrincipalPointColIdx],
          projectionParams[Fisheye624::kPrincipalPointRowIdx]};
  }
  // Intentionally skipping default to raise a compile error when new models are added.
  assert(false);
  // return 0s to remove compiler warning
  return Eigen::Vector2d::Zero();
}

Eigen::Vector2d CameraProjectionModel::project(const Eigen::Vector3d& p) const {
  switch (modelName) {
    case ModelType::KannalaBrandtK3:
      return KannalaBrandtK3Projection::project(p, projectionParams);
    case ModelType::Fisheye624:
      return Fisheye624::project(p, projectionParams);
      // Intentionally skipping default to raise a compile error when new models are added.
  }
  assert(false);
  // return 0s to remove compiler warning
  return Eigen::Vector2d::Zero();
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
  // return 0s to remove compiler warning
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d LinearRectificationModel::compensateForSystematicErrorFromMeasurement(
    const Eigen::Vector3d& v_raw) const {
  return rectificationMatrix.inverse() * (v_raw - bias);
}

Eigen::Vector3d LinearRectificationModel::distortWithSystematicError(
    const Eigen::Vector3d& v_compensated) const {
  return rectificationMatrix * v_compensated + bias;
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

namespace utils {
// Provide an alternative to std::regex_replace
void ReplaceStringInPlace(
    std::string& subject,
    const std::string& search,
    const std::string& replace) {
  size_t pos = 0;
  while ((pos = subject.find(search, pos)) != std::string::npos) {
    subject.replace(pos, search.length(), replace);
    pos += replace.length();
  }
}

// Replace Document json with an array version if we have a string
// Note: Online calibration data is saved with a string,
//  we can convert it to an array with this code.
// See DeviceModelTests.cpp to know more about the JSON message format.
fb_rapidjson::Value& NormalizeToArrayIfString(
    fb_rapidjson::Value& value,
    fb_rapidjson::Document& doc) {
  if (value.IsString()) {
    // Convert substring to Json object
    std::string temp = value.GetString();
    // Adjust string to be a valid JSON
    std::replace(temp.begin(), temp.end(), '\'', '\"');
    ReplaceStringInPlace(temp, "True", "true");
    ReplaceStringInPlace(temp, "False", "false");
    fb_rapidjson::Document doc_temp;
    doc_temp.Parse(temp.c_str());
    value.CopyFrom(doc_temp, doc.GetAllocator());
  }
  return value;
}
} // namespace utils

DeviceModel DeviceModel::fromJson(const fb_rapidjson::Document& json) {
  DeviceModel calib;
  {
    // Use a local Json Document copy if we need to overwrite some fields
    fb_rapidjson::Document jsonCpy;
    jsonCpy.CopyFrom(json, jsonCpy.GetAllocator());

    if (json.FindMember("CameraCalibrations") != json.MemberEnd()) {
      const fb_rapidjson::Value& v =
          utils::NormalizeToArrayIfString(jsonCpy["CameraCalibrations"], jsonCpy);
      for (const auto& camJson : v.GetArray()) {
        CameraCalibration camCalib = parseCameraCalibFromJson(camJson);
        auto& ref = calib.cameraCalibs_[camCalib.label];
        ref = std::move(camCalib);
      }
    }
    if (json.FindMember("ImuCalibrations") != json.MemberEnd()) {
      const fb_rapidjson::Value& v =
          utils::NormalizeToArrayIfString(jsonCpy["ImuCalibrations"], jsonCpy);
      for (const auto& imuJson : v.GetArray()) {
        ImuCalibration imuCalib = parseImuCalibFromJson(imuJson);
        auto& ref = calib.imuCalibs_[imuCalib.label];
        ref = std::move(imuCalib);
      }
    }
  }

  if (json.FindMember("DeviceClassInfo") != json.MemberEnd()) {
    std::string deviceSubtype = json["DeviceClassInfo"]["BuildVersion"].GetString();
    if (json.FindMember("MagCalibrations") != json.MemberEnd()) {
      for (const auto& magnetometerJson : json["MagCalibrations"].GetArray()) {
        MagnetometerCalibration magnetometerCalib =
            parseMagnetometerCalibrationFromJson(magnetometerJson, calib, deviceSubtype);
        auto& ref = calib.magnetometerCalibs_[magnetometerCalib.label];
        ref = std::move(magnetometerCalib);
      }
    }
    if (json.FindMember("BaroCalibrations") != json.MemberEnd()) {
      for (const auto& barometerJson : json["BaroCalibrations"].GetArray()) {
        BarometerCalibration barometerCalib =
            parseBarometerCalibrationFromJson(barometerJson, calib, deviceSubtype);
        auto& ref = calib.barometerCalibs_[barometerCalib.label];
        ref = std::move(barometerCalib);
      }
    }
    if (json.FindMember("MicCalibrations") != json.MemberEnd()) {
      for (const auto& microphoneJson : json["MicCalibrations"].GetArray()) {
        MicrophoneCalibration microphoneCalib =
            parseMicrophoneCalibrationFromJson(microphoneJson, calib, deviceSubtype);
        auto& ref = calib.microphoneCalibs_[microphoneCalib.label];
        ref = std::move(microphoneCalib);
      }
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
      if (camParams[Fisheye624::kPrincipalPointColIdx] * 2 >
          newWidth) { // We need to rescale calibration parameters

        // Assume the resolution change follows the following steps:
        // - centered cropping -> sensor pixel binning
        const double rescaleFactor = std::floor(
            nativeResolution / static_cast<double>(newWidth)); // binning can only be an integer
        const double halfCroppedSize = (nativeResolution - newWidth * rescaleFactor) / 2.0;
        camParams[Fisheye624::kPrincipalPointColIdx] -= halfCroppedSize;
        camParams[Fisheye624::kPrincipalPointRowIdx] -= halfCroppedSize;

        camParams[Fisheye624::kFocalXIdx] /= rescaleFactor;
        camParams[Fisheye624::kPrincipalPointColIdx] /= rescaleFactor;
        camParams[Fisheye624::kPrincipalPointRowIdx] /= rescaleFactor;

        if (cameraCalib.validRadius != -1) {
          cameraCalib.validRadius /= rescaleFactor;
        }

        updatedCameraCalibs_.insert(label);
        return true;
      }
    } else if (label == "camera-et-left" || label == "camera-et-right") {
      Eigen::VectorXd& camParams = cameraCalib.projectionModel.projectionParams;
      // Testing if principal point is appropriate for this image width
      if (camParams[KannalaBrandtK3Projection::kPrincipalPointColIdx] * 2 >
          newWidth) { // We need to rescale calibration parameters

        // Assume the resolution change following a linear rescaling
        const double rescaleFactor = std::floor(nativeResolution / static_cast<double>(newWidth));
        camParams[KannalaBrandtK3Projection::kFocalXIdx] /= rescaleFactor;
        camParams[KannalaBrandtK3Projection::kFocalYIdx] /= rescaleFactor;
        camParams[KannalaBrandtK3Projection::kPrincipalPointColIdx] /= rescaleFactor;
        camParams[KannalaBrandtK3Projection::kPrincipalPointRowIdx] /= rescaleFactor;

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
