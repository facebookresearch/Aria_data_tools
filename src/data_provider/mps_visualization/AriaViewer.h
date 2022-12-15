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

#include <deque>
#include "data_provider/AriaVrsDataProvider.h"
#include "eyeGazeReader.h"
#include "models/DeviceModel.h"
#include "utils.h"

namespace ark::datatools::visualization {

class AriaViewer {
 public:
  AriaViewer(
      datatools::dataprovider::AriaDataProvider* dataProvider,
      int width,
      int height,
      const std::string& name = "AriaEyeGazeViewer");
  ~AriaViewer() = default;
  void run();

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

  bool isDataChanged(const vrs::StreamId& streamId) {
    return dataChangedMap_[streamId.getTypeId()][streamId.getInstanceId()];
  }

  void setDataChanged(bool dataChanged, const vrs::StreamId& streamId) {
    dataChangedMap_[streamId.getTypeId()][streamId.getInstanceId()] = dataChanged;
  }
  void setCameraImageBuffer(const std::vector<uint8_t>& buffer, const vrs::StreamId& streamId) {
    cameraImageBufferMap_[streamId.getTypeId()][streamId.getInstanceId()] = buffer;
  }

  std::pair<double, double> initDataStreams(const std::filesystem::path& eyeTrackingFilepath);
  // read data until currentTimestampSec
  bool readData(double currentTimestampSec);

 private:
  const int width_, height_;
  const std::string name_;
  bool isPlaying_ = false;
  float playbackSpeedFactor_ = 1;
  std::mutex dataMutex_;

  // Store if RecordableTypeId has been changed
  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, bool>> dataChangedMap_;
  // Current image data buffer per RecordableTypeID to be shown
  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, std::vector<uint8_t>>>
      cameraImageBufferMap_;

 public:
  // Interface to store temporally sorted EyeGaze data record
  using EyeGazeDataRecords = std::map<std::chrono::microseconds, std::pair<Eigen::Vector3d, float>>;

 private:
  // Store all record for search (timestamp sorted)
  EyeGazeDataRecords eyeGazeData_;
  // Last current valid EyeGaze recording
  EyeGazeDataRecords::mapped_type lastEyeGazeRecord_;
  // A rolling buffer history of EyeGaze yaw, pitch recordings
  std::deque<Eigen::Vector2d> eyeGazeHistory_;

  // Aria VRS data provider
  datatools::dataprovider::AriaDataProvider* dataProvider_;
  // Store current Timestamp relative to the Aria sequence we are at
  std::int64_t currentTimestamp_ = 0;
};

} // namespace ark::datatools::visualization
