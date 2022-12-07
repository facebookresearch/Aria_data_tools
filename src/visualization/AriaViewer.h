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

#include <pangolin/pangolin.h>
#include "data_provider/AriaVrsDataProvider.h"
#include "models/DeviceModel.h"
#include "utils.h"

namespace ark {
namespace datatools {
namespace visualization {

class AriaViewer {
 public:
  AriaViewer(
      datatools::dataprovider::AriaDataProvider* dataProvider,
      int width,
      int height,
      const std::string& name = "AriaViewer",
      int id = 0);
  ~AriaViewer() = default;
  void run();
  void drawTraj();
  void drawRigs(
      bool showRig3D,
      bool showLeftCam3D,
      bool showRightCam3D,
      bool showRgbCam3D,
      int camSparsity);

  void setPose(const std::optional<Sophus::SE3d>& T_World_ImuLeft);
  void setEyetracksOnRgbImage(const std::optional<Eigen::Vector2f>& eyetrackOnRgbImage);
  void setSpeechToText(
      const std::optional<ark::datatools::dataprovider::SpeechToTextDatum>& speechToText);

  std::mutex& getDataMutex() {
    return dataMutex_;
  }

  std::thread runInThread() {
    return std::thread(&AriaViewer::run, this);
  }

  bool isPlaying() const {
    return isPlaying_;
  }

  float getPlaybackSpeedFactor() const {
    return playbackSpeedFactor_;
  }

  void setDataChanged(bool dataChanged, const vrs::StreamId& streamId) {
    dataChangedMap_[streamId.getTypeId()][streamId.getInstanceId()] = dataChanged;
  }
  void setCameraImageBuffer(const std::vector<uint8_t>& buffer, const vrs::StreamId& streamId) {
    cameraImageBufferMap_[streamId.getTypeId()][streamId.getInstanceId()] = buffer;
  }

  bool isDataChanged(const vrs::StreamId& streamId) {
    return dataChangedMap_[streamId.getTypeId()][streamId.getInstanceId()];
  }

  void setImuDataChunk(
      const vrs::StreamId& streamId,
      const std::vector<Eigen::Vector3f>& accMSec2,
      const std::vector<Eigen::Vector3f>& gyroRadSec) {
    setDataChanged(true, streamId);
    accMSec2Map_[streamId.getTypeId()][streamId.getInstanceId()] = accMSec2;
    gyroRadSecMap_[streamId.getTypeId()][streamId.getInstanceId()] = gyroRadSec;
  }

  void setMagnetometerChunk(
      const vrs::StreamId& streamId,
      const std::vector<Eigen::Vector3f>& magTesla) {
    if (magTesla.empty()) {
      return;
    }
    setDataChanged(true, streamId);
    magTesla_ = magTesla;
  }

  void setBarometerChunk(
      const vrs::StreamId& streamId,
      const std::vector<float>& temperature,
      const std::vector<float>& pressure) {
    if (temperature.size() && pressure.size()) {
      setDataChanged(true, streamId);
      temperature_ = temperature;
      pressure_ = pressure;
    }
  }

  void setAudioChunk(const vrs::StreamId& streamId, const std::vector<std::vector<float>>& audio) {
    if (audio.empty()) {
      return;
    }
    setDataChanged(true, streamId);
    audio_ = audio;
  }

  std::pair<double, double> initDataStreams();
  // read data until currentTimestampSec
  bool readData(double currentTimestampSec);

 private:
  const int width_, height_;
  const std::string name_;
  const int id_;
  bool isPlaying_ = false;
  float playbackSpeedFactor_ = 1;
  std::vector<Sophus::SE3d> T_World_ImuLeft_;
  std::vector<Eigen::Vector2f> eyetracksOnRgbImage_;
  std::optional<ark::datatools::dataprovider::SpeechToTextDatum> speechToText_;
  Sophus::SE3d T_Viewer_World_;
  bool hasFirstPose_ = false;
  std::mutex dataMutex_;

  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, bool>> dataChangedMap_;
  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, std::vector<uint8_t>>>
      cameraImageBufferMap_;
  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, Sophus::SE3d>>
      T_ImuLeft_cameraMap_;

  // current accMSec2 and gyroRadSec chunks.
  std::unordered_map<
      vrs::RecordableTypeId,
      std::unordered_map<uint16_t, std::vector<Eigen::Vector3f>>>
      accMSec2Map_;
  std::unordered_map<
      vrs::RecordableTypeId,
      std::unordered_map<uint16_t, std::vector<Eigen::Vector3f>>>
      gyroRadSecMap_;

  // current audio chunks.
  std::vector<std::vector<float>> audio_;

  // current barometer chunks.
  std::vector<float> temperature_;
  std::vector<float> pressure_;

  // current magnetometer chunks.
  std::vector<Eigen::Vector3f> magTesla_;

  datatools::dataprovider::AriaDataProvider* dataProvider_;
  datatools::sensors::DeviceModel deviceModel_;
};
} // namespace visualization
} // namespace datatools
} // namespace ark
