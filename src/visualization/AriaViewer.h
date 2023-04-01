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

#include "AriaViewerBase.h"
#include "data_provider/speech_to_text_datum.h"
#include "utils.h"

namespace ark {
namespace datatools {
namespace visualization {

class AriaViewer : public AriaViewerBase {
 public:
  AriaViewer(
      datatools::dataprovider::AriaDataProvider* dataProvider,
      int width,
      int height,
      const std::string& name = "AriaViewer",
      int id = 0);

  ~AriaViewer() override = default;

  void run() override;

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

  std::pair<double, double> initDataStreams(
      const std::vector<vrs::StreamId>& kImageStreamIds,
      const std::vector<vrs::StreamId>& kImuStreamIds = {},
      const std::vector<vrs::StreamId>& kDataStreams = {}) override;

 private:
  std::vector<Sophus::SE3d> T_World_ImuLeft_;
  std::vector<Eigen::Vector2f> eyetracksOnRgbImage_;
  std::optional<ark::datatools::dataprovider::SpeechToTextDatum> speechToText_;
  Sophus::SE3d T_Viewer_World_;
  bool hasFirstPose_ = false;

  // Save transformation to move data to IMU Left (Aria Pilot dataset Reference system)
  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, Sophus::SE3d>>
      T_ImuLeft_cameraMap_;
};
} // namespace visualization
} // namespace datatools
} // namespace ark
