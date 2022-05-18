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
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include "players/AriaAudioPlayer.h"
#include "players/AriaBarometerPlayer.h"
#include "players/AriaBluetoothBeaconPlayer.h"
#include "players/AriaGpsPlayer.h"
#include "players/AriaImageSensorPlayer.h"
#include "players/AriaMotionSensorPlayer.h"
#include "players/AriaTimeSyncPlayer.h"
#include "players/AriaWifiBeaconPlayer.h"

#include "data_provider/AriaDataProvider.h"

#include <filesystem>
#include <mutex>
#include <unordered_map>

namespace ark {
namespace datatools {
namespace dataprovider {

class AriaVrsDataProvider : public AriaDataProvider {
 public:
  // vrs::RecordFileReader wrapper functions
  bool openFile(const std::string& vrsFilePath);
  std::set<vrs::StreamId> getStreamsInFile();
  bool readAllRecords();
  bool readFirstConfigurationRecord(const vrs::StreamId& streamId);
  double getFirstTimestampSec(const vrs::StreamId& streamId, const vrs::Record::Type& type);
  const vrs::IndexRecord::RecordInfo* getRecordByTime(
      const vrs::StreamId& streamId,
      const vrs::Record::Type& type,
      double timestampSec);
  bool readRecordsByTime(const vrs::Record::Type& type, double timestampSec);
  bool readRecordByTime(vrs::StreamId streamId, const vrs::Record::Type& type, double timestampSec);
  bool readRecord(const vrs::IndexRecord::RecordInfo& record);
  const vrs::IndexRecord::RecordInfo* getLastRecord(
      vrs::StreamId streamId,
      vrs::Record::Type recordType);

  // Data record specific vrs::RecordFileReader wrapper functions for convenience
  double getFirstDataRecordTimestampSec(const vrs::StreamId& streamId);
  const vrs::IndexRecord::RecordInfo* getDataRecordByTime(
      const vrs::StreamId& streamId,
      double timestampSec);
  bool readDataRecordsByTime(double timestampSec);
  bool readDataRecordByTime(vrs::StreamId streamId, double timestampSec);
  const vrs::IndexRecord::RecordInfo* getLastDataRecord(vrs::StreamId streamId);

  void setSlamLeftCameraPlayer();
  void setSlamRightCameraPlayer();
  void setRgbCameraPlayer();
  void setEyeCameraPlayer();
  void setImuRightPlayer();
  void setImuLeftPlayer();
  void setMagnetometerPlayer();
  void setWifiBeaconPlayer();
  void setAudioPlayer();
  void setGpsPlayer();
  void setBarometerPlayer();
  void setTimeSyncPlayer();
  void setStreamPlayer(const vrs::StreamId& streamId) override;

  const AriaImageSensorPlayer* getSlamLeftCameraPlayer();
  const AriaImageSensorPlayer* getSlamRightCameraPlayer();
  const AriaImageSensorPlayer* getRgbCameraPlayer();
  const AriaImageSensorPlayer* getEyeCameraPlayer();
  const AriaMotionSensorPlayer* getImuRightPlayer();
  const AriaMotionSensorPlayer* getImuLeftPlayer();
  const AriaMotionSensorPlayer* getMagnetometerPlayer();
  const AriaWifiBeaconPlayer* getWifiBeaconPlayer();
  const AriaAudioPlayer* getAudioPlayer();
  const AriaBluetoothBeaconPlayer* getBluetoothBeaconPlayer();
  const AriaGpsPlayer* getGpsPlayer();
  const AriaBarometerPlayer* getBarometerPlayer();
  const AriaTimeSyncPlayer* getTimeSyncPlayer();
  const AriaImageSensorPlayer* getImageSensorPlayer(const vrs::StreamId& streamId) const;
  const AriaMotionSensorPlayer* getMotionSensorPlayer(const vrs::StreamId& streamId) const;
  double getNextTimestampSec(const vrs::StreamId& streamId) const;

  const std::unordered_map<
      vrs::RecordableTypeId,
      std::unordered_map<uint16_t, std::unique_ptr<AriaImageSensorPlayer>>>&
  getImagePlayers() {
    return imagePlayers_;
  }

  const std::unordered_map<
      vrs::RecordableTypeId,
      std::unordered_map<uint16_t, std::unique_ptr<AriaMotionSensorPlayer>>>&
  getMotionPlayers() {
    return motionPlayers_;
  }

  void setVerbose(bool verbose);

  // Override functions for AriaDataProvider
  bool open(const std::string& vrsPath, const std::string& posePath = "") override;
  bool tryFetchNextData(const vrs::StreamId& streamId, double currentTimestampSec) override;
  void* getImageBuffer(const vrs::StreamId& streamId) const override;
  uint32_t getImageWidth(const vrs::StreamId& streamId) const override;
  uint32_t getImageHeight(const vrs::StreamId& streamId) const override;
  double getFastestNominalRateHz() override;
  double getFirstTimestampSec() override;
  std::optional<Sophus::SE3d> getPose() const override;
  bool loadPosesFromCsv(const std::string& posePath) override;
  bool atLastRecords() override;
  bool loadDeviceModel() override;

 private:
  void createImagePlayer(const vrs::StreamId& streamId);
  void createMotionPlayer(const vrs::StreamId& streamId);
  void createWifiBeaconPlayer(const vrs::StreamId& streamId);
  void createAudioPlayer(const vrs::StreamId& streamId);
  void createBluetoothBeaconPlayer(const vrs::StreamId& streamId);
  void createGpsPlayer(const vrs::StreamId& streamId);
  void createBarometerPlayer(const vrs::StreamId& streamId);
  void createTimeSyncPlayer(const vrs::StreamId& streamId);

  vrs::RecordFileReader reader_;
  std::mutex readerMutex_;

  std::unordered_map<
      vrs::RecordableTypeId,
      std::unordered_map<uint16_t, std::unique_ptr<AriaImageSensorPlayer>>>
      imagePlayers_;
  std::unordered_map<
      vrs::RecordableTypeId,
      std::unordered_map<uint16_t, std::unique_ptr<AriaMotionSensorPlayer>>>
      motionPlayers_;
  std::unique_ptr<AriaWifiBeaconPlayer> wifiBeaconPlayer_;
  std::unique_ptr<AriaAudioPlayer> audioPlayer_;
  std::unique_ptr<AriaBluetoothBeaconPlayer> bluetoothBeaconPlayer_;
  std::unique_ptr<AriaGpsPlayer> gpsPlayer_;
  std::unique_ptr<AriaBarometerPlayer> barometerPlayer_;
  std::unique_ptr<AriaTimeSyncPlayer> timeSyncPlayer_;

  std::unordered_map<vrs::RecordableTypeId, std::unordered_map<uint16_t, bool>>
      isFirstConfigRecordRead_;
};
} // namespace dataprovider
} // namespace datatools
} // namespace ark
