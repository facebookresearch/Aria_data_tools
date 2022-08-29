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

#include <sophus/se3.hpp>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "data_provider/AriaDataProvider.h"

namespace pangolin {
class TypedImage;
}

namespace ark {
namespace datatools {
namespace dataprovider {

struct StreamInFolder {
  std::string folderPath;
  std::vector<std::string> filePaths;
  std::vector<int64_t> timestampsNs;
  size_t currentFileIndex = 0;
  std::unordered_map<int64_t, std::string> timestampNsToPath;
};

struct ImageStreamInFolder : public StreamInFolder {
  std::shared_ptr<pangolin::TypedImage> image;
};

class AriaFolderDataProvider : public AriaDataProvider {
 public:
  bool open(const std::string& folderPath, const std::string& posePath = "") override;
  void setStreamPlayer(const vrs::StreamId& streamId) override;
  bool tryFetchNextData(
      const vrs::StreamId& streamId,
      double currentTimestampSec = std::numeric_limits<double>::max()) override;
  void* getImageBuffer(const vrs::StreamId& streamId) const override;
  double getFastestNominalRateHz() override {
    std::cout << "getFastestNominalRateHz is not implemented yet!" << std::endl;
    return 10;
  }
  uint32_t getImageWidth(const vrs::StreamId& streamId) const override {
    std::cout << "getImageWidth is not implemented yet!" << std::endl;
    if (streamId.getTypeId() == vrs::RecordableTypeId::RgbCameraRecordableClass) {
      return 1408;
    }
    return 640;
  }
  uint32_t getImageHeight(const vrs::StreamId& streamId) const override {
    std::cout << "getImageHeight is not implemented yet!" << std::endl;
    if (streamId.getTypeId() == vrs::RecordableTypeId::RgbCameraRecordableClass) {
      return 1408;
    }
    return 480;
  }
  // imu data
  Eigen::Vector3f getMotionAccelData(const vrs::StreamId& streamId) const override {
    std::cout << "getMotionAccelData is not implemented yet!" << std::endl;
    return {};
  }
  Eigen::Vector3f getMotionGyroData(const vrs::StreamId& streamId) const override {
    std::cout << "getMotionGyroData is not implemented yet!" << std::endl;
    return {};
  }
  // barometer data
  double getBarometerPressure() const override {
    std::cout << "getBarometerPressure is not implemented yet!" << std::endl;
    return 0;
  }
  double getBarometerTemperature() const override {
    std::cout << "getBarometerTemperature is not implemented yet!" << std::endl;
    return 0;
  }
  // magnetometer data
  Eigen::Vector3f getMagnetometerData() const override {
    std::cout << "getMagnetometerData is not implemented yet!" << std::endl;
    return {};
  }
  // audio data
  const std::vector<int32_t>& getAudioData() const override {
    std::cout << "getAudioData is not implemented yet!" << std::endl;
    return {};
  }
  uint8_t getAudioNumChannels() const override {
    std::cout << "getAudioNumChannels is not implemented yet!" << std::endl;
    return 0;
  }
  double getFirstTimestampSec() override;
  std::optional<Sophus::SE3d> getPose() const override;
  bool atLastRecords() override;
  bool loadPosesFromCsv(const std::string& posePath) override;
  // Load device model by parsing metadata file
  bool loadDeviceModel() override;
  bool streamExistsInSource(const vrs::StreamId& streamId) override;

  void setImagePlayerVerbose(const vrs::StreamId& streamId, bool verbose) override {
    std::cout << "setImagePlayerVerbose is not implemented yet!" << std::endl;
  }
  void setMotionPlayerVerbose(const vrs::StreamId& streamId, bool verbose) override {
    std::cout << "setMotionPlayerVerbose is not implemented yet!" << std::endl;
  }
  void setWifiBeaconPlayerVerbose(bool verbose) override {
    std::cout << "setWifiBeaconPlayerVerbose is not implemented yet!" << std::endl;
  }
  void setAudioPlayerVerbose(bool verbose) override {
    std::cout << "setAudioPlayerVerbose is not implemented yet!" << std::endl;
  }
  void setBluetoothBeaconPlayerVerbose(bool verbose) override {
    std::cout << "setBluetoothBeaconPlayerVerbose is not implemented yet!" << std::endl;
  }
  void setGpsPlayerVerbose(bool verbose) override {
    std::cout << "setGpsPlayerVerbose is not implemented yet!" << std::endl;
  }
  void setBarometerPlayerVerbose(bool verbose) override {
    std::cout << "setBarometerPlayerVerbose is not implemented yet!" << std::endl;
  }
  void setTimeSyncPlayerVerbose(bool verbose) override {
    std::cout << "setTimeSyncPlayerVerbose is not implemented yet!" << std::endl;
  }

 private:
  ImageStreamInFolder* getImageStream(const vrs::StreamId& streamId) const;

  std::string trajectoryCsvPath_, metadataPath_;
  std::unordered_map<
      vrs::RecordableTypeId,
      std::unordered_map<uint16_t, std::unique_ptr<StreamInFolder>>>
      streamsInFolder_;

  std::map<int64_t, Sophus::SE3d> imuLeftPoses_;
  bool hasPoses_ = false;
};
} // namespace dataprovider
} // namespace datatools
} // namespace ark
