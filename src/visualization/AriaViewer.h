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
  AriaViewer(const datatools::dataprovider::AriaDataProvider* dataProvider, int width, int height);
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

  std::mutex& getDataMutex() {
    return dataMutex_;
  }

  bool isPlaying() const {
    return isPlaying_;
  }

  float getPlaybackSpeedFactor() const {
    return playbackSpeedFactor_;
  }

  void setCameraImageChanged(bool cameraImageChanged, const vrs::StreamId& streamId) {
    cameraImageChangedMap_[streamId.getTypeId()][streamId.getInstanceId()] = cameraImageChanged;
  }

 private:
  const int width_, height_;
  bool isPlaying_ = false;
  float playbackSpeedFactor_ = 1;
  std::vector<Sophus::SE3d> T_World_ImuLeft_;
  std::vector<Eigen::Vector2f> eyetracksOnRgbImage_;
  Sophus::SE3d T_Viewer_World_;
  bool hasFirstPose_ = false;
  bool hasPose_ = false;
  std::mutex dataMutex_;

  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, bool>>
      cameraImageChangedMap_;
  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, Sophus::SE3d>>
      T_ImuLeft_cameraMap_;

  const datatools::dataprovider::AriaDataProvider* dataProvider_ = nullptr;
  const datatools::sensors::DeviceModel deviceModel_;
};
} // namespace visualization
} // namespace datatools
} // namespace ark
