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

#include "AriaVrsDataProvider.h"
#include "utils.h"

#include <fmt/core.h>
#include <fmt/ostream.h>

namespace {

void printDataLayout(const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout) {
  fmt::print(
      "{:.3f} {} record, {} [{}]\n",
      r.timestamp,
      toString(r.recordType),
      r.streamId.getName(),
      r.streamId.getNumericName());
  dataLayout.printLayoutCompact(std::cout, "  ");
}

auto kAudioCallback = [](const vrs::CurrentRecord&, std::vector<int32_t>&, bool) { return true; };
auto kBarometerCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, dataLayout);
      }
      return true;
    };
auto kBluetoothBeaconCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, dataLayout);
      }
      return true;
    };
auto kGpsCallback = [](const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose) {
  if (verbose) {
    printDataLayout(r, dataLayout);
  }
  return true;
};
auto kImageCallback = [](const vrs::CurrentRecord&, std::vector<uint8_t>&, bool) { return true; };
auto kMotionCallback = [](const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose) {
  if (verbose) {
    printDataLayout(r, dataLayout);
  }
  return true;
};
auto kTimeSyncCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, dataLayout);
      }
      return true;
    };
auto kWifiBeaconCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, dataLayout);
      }
      return true;
    };
auto kPoseCallback = [](const vrs::CurrentRecord& r, vrs::DataLayout& dataLayout, bool verbose) {
  if (verbose) {
    printDataLayout(r, dataLayout);
  }
  return true;
};
constexpr const char* kTrajectoryPathSuffix = "location/trajectory.csv";
constexpr const char* kEyetrackingPathSuffix = "eyetracking/et_in_rgb_stream.csv";
constexpr const char* kSpeechToTextPathSuffix = "speech2text/speech_aria_domain.csv";
const int kAriaNativeRgbResolution = 2880;
const int kAriaNativeEtResolution = 640;
}; // namespace

namespace ark {
namespace datatools {
namespace dataprovider {

bool AriaVrsDataProvider::openFile(const std::string& vrsFilePath) {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  return reader_.openFile(vrsFilePath) == vrs::SUCCESS;
}

std::set<vrs::StreamId> AriaVrsDataProvider::getStreamsInFile() {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  return reader_.getStreams();
}

bool AriaVrsDataProvider::readAllRecords() {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  return reader_.readAllRecords() == vrs::SUCCESS;
}

bool AriaVrsDataProvider::readFirstConfigurationRecord(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  const auto& recordableTypeId = streamId.getTypeId();
  const auto& instanceId = streamId.getInstanceId();
  if (!isFirstConfigRecordRead_[recordableTypeId][instanceId]) {
    isFirstConfigRecordRead_[recordableTypeId][instanceId] =
        reader_.readFirstConfigurationRecord(streamId);
  }
  return isFirstConfigRecordRead_[recordableTypeId][instanceId];
}

double AriaVrsDataProvider::getFirstTimestampSec(
    const vrs::StreamId& streamId,
    const vrs::Record::Type& type) {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  if (reader_.getRecordCount(streamId, type) == 0) {
    return 0;
  }
  return reader_.getRecord(streamId, type, 0)->timestamp;
}

double AriaVrsDataProvider::getFirstDataRecordTimestampSec(const vrs::StreamId& streamId) {
  return getFirstTimestampSec(streamId, vrs::Record::Type::DATA);
}

const vrs::IndexRecord::RecordInfo* AriaVrsDataProvider::getRecordByTime(
    const vrs::StreamId& streamId,
    const vrs::Record::Type& type,
    double timestampSec) {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  return reader_.getRecordByTime(streamId, type, timestampSec);
}

const vrs::IndexRecord::RecordInfo* AriaVrsDataProvider::getDataRecordByTime(
    const vrs::StreamId& streamId,
    double timestampSec) {
  return getRecordByTime(streamId, vrs::Record::Type::DATA, timestampSec);
}

bool AriaVrsDataProvider::readRecordsByTime(const vrs::Record::Type& type, double timestampSec) {
  for (auto& players : imagePlayers_) {
    for (auto& player : players.second) {
      auto& imagePlayer = player.second;
      if (!readRecordByTime(imagePlayer->getStreamId(), type, timestampSec)) {
        return false;
      }
    }
  }
  for (auto& players : motionPlayers_) {
    for (auto& player : players.second) {
      auto& motionPlayer = player.second;
      if (!readRecordByTime(motionPlayer->getStreamId(), type, timestampSec)) {
        return false;
      }
    }
  }
  if (!readRecordByTime(wifiBeaconPlayer_->getStreamId(), type, timestampSec)) {
    return false;
  }
  if (!readRecordByTime(audioPlayer_->getStreamId(), type, timestampSec)) {
    return false;
  }
  if (!readRecordByTime(bluetoothBeaconPlayer_->getStreamId(), type, timestampSec)) {
    return false;
  }
  if (!readRecordByTime(gpsPlayer_->getStreamId(), type, timestampSec)) {
    return false;
  }
  if (!readRecordByTime(barometerPlayer_->getStreamId(), type, timestampSec)) {
    return false;
  }
  if (!readRecordByTime(timeSyncPlayer_->getStreamId(), type, timestampSec)) {
    return false;
  }
  if (!readRecordByTime(posePlayer_->getStreamId(), type, timestampSec)) {
    return false;
  }
  return true;
}

bool AriaVrsDataProvider::readDataRecordsByTime(double timestampSec) {
  return readRecordsByTime(vrs::Record::Type::DATA, timestampSec);
}

bool AriaVrsDataProvider::readRecordByTime(
    vrs::StreamId streamId,
    const vrs::Record::Type& type,
    double timestampSec) {
  auto record = getRecordByTime(streamId, type, timestampSec);
  if (record == nullptr) {
    fmt::print(
        "Can't read record at timestamp {} for stream: {}, {}.\n",
        timestampSec,
        streamId.getNumericName(),
        streamId.getName());
    return false;
  }
  return readRecord(*record);
}

bool AriaVrsDataProvider::readDataRecordByTime(vrs::StreamId streamId, double timestampSec) {
  return readRecordByTime(streamId, vrs::Record::Type::DATA, timestampSec);
}

const vrs::IndexRecord::RecordInfo* AriaVrsDataProvider::getLastRecord(
    vrs::StreamId streamId,
    vrs::Record::Type type) {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  return reader_.getLastRecord(streamId, type);
}

const vrs::IndexRecord::RecordInfo* AriaVrsDataProvider::getLastDataRecord(vrs::StreamId streamId) {
  return getLastRecord(streamId, vrs::Record::Type::DATA);
}

bool AriaVrsDataProvider::readRecord(const vrs::IndexRecord::RecordInfo& record) {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  return reader_.readRecord(record) == vrs::SUCCESS;
}

bool AriaVrsDataProvider::streamExistsInSource(const vrs::StreamId& streamId) {
  auto streamsInFile = getStreamsInFile();
  return streamsInFile.find(streamId) != streamsInFile.end();
}

void AriaVrsDataProvider::createImagePlayer(const vrs::StreamId& streamId) {
  std::unique_ptr<AriaImageSensorPlayer> imagePlayer =
      std::make_unique<AriaImageSensorPlayer>(streamId);
  imagePlayer->setCallback(kImageCallback);
  imagePlayers_[streamId.getTypeId()][streamId.getInstanceId()] = std::move(imagePlayer);
}

void AriaVrsDataProvider::createMotionPlayer(const vrs::StreamId& streamId) {
  std::unique_ptr<AriaMotionSensorPlayer> motionPlayer =
      std::make_unique<AriaMotionSensorPlayer>(streamId);
  motionPlayer->setCallback(kMotionCallback);
  motionPlayers_[streamId.getTypeId()][streamId.getInstanceId()] = std::move(motionPlayer);
}

void AriaVrsDataProvider::createWifiBeaconPlayer(const vrs::StreamId& streamId) {
  wifiBeaconPlayer_ = std::make_unique<AriaWifiBeaconPlayer>(streamId);
  wifiBeaconPlayer_->setCallback(kWifiBeaconCallback);
}

void AriaVrsDataProvider::createAudioPlayer(const vrs::StreamId& streamId) {
  audioPlayer_ = std::make_unique<AriaAudioPlayer>(streamId);
  audioPlayer_->setCallback(kAudioCallback);
}

void AriaVrsDataProvider::createBluetoothBeaconPlayer(const vrs::StreamId& streamId) {
  bluetoothBeaconPlayer_ = std::make_unique<AriaBluetoothBeaconPlayer>(streamId);
  bluetoothBeaconPlayer_->setCallback(kBluetoothBeaconCallback);
}

void AriaVrsDataProvider::createGpsPlayer(const vrs::StreamId& streamId) {
  gpsPlayer_ = std::make_unique<AriaGpsPlayer>(streamId);
  gpsPlayer_->setCallback(kGpsCallback);
}

void AriaVrsDataProvider::createBarometerPlayer(const vrs::StreamId& streamId) {
  barometerPlayer_ = std::make_unique<AriaBarometerPlayer>(streamId);
  barometerPlayer_->setCallback(kBarometerCallback);
}

void AriaVrsDataProvider::createTimeSyncPlayer(const vrs::StreamId& streamId) {
  timeSyncPlayer_ = std::make_unique<AriaTimeSyncPlayer>(streamId);
  timeSyncPlayer_->setCallback(kTimeSyncCallback);
}

void AriaVrsDataProvider::createPosePlayer(const vrs::StreamId& streamId) {
  posePlayer_ = std::make_unique<AriaPosePlayer>(streamId);
  posePlayer_->setCallback(kPoseCallback);
}

void AriaVrsDataProvider::setSlamLeftCameraPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 1));
}

void AriaVrsDataProvider::setSlamRightCameraPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 2));
}

void AriaVrsDataProvider::setRgbCameraPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1));
}

void AriaVrsDataProvider::setEyeCameraPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::EyeCameraRecordableClass, 1));
}

void AriaVrsDataProvider::setImuRightPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 1));
}

void AriaVrsDataProvider::setImuLeftPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 2));
}

void AriaVrsDataProvider::setMagnetometerPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamMagnetometerData, 1));
}

void AriaVrsDataProvider::setWifiBeaconPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::WifiBeaconRecordableClass, 1));
}

void AriaVrsDataProvider::setBluetoothBeaconPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::BluetoothBeaconRecordableClass, 1));
}

void AriaVrsDataProvider::setAudioPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::StereoAudioRecordableClass, 1));
}

void AriaVrsDataProvider::setGpsPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::GpsRecordableClass, 1));
}

void AriaVrsDataProvider::setBarometerPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::BarometerRecordableClass, 1));
}

void AriaVrsDataProvider::setTimeSyncPlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::TimeRecordableClass, 1));
}

void AriaVrsDataProvider::setPosePlayer() {
  setStreamPlayer(vrs::StreamId(vrs::RecordableTypeId::PoseRecordableClass, 1));
}

void AriaVrsDataProvider::setStreamPlayer(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readerLock(readerMutex_);
  vrs::StreamPlayer* streamPlayer = nullptr;
  switch (streamId.getTypeId()) {
    case vrs::RecordableTypeId::SlamCameraData:
    case vrs::RecordableTypeId::RgbCameraRecordableClass:
    case vrs::RecordableTypeId::EyeCameraRecordableClass:
      createImagePlayer(streamId);
      streamPlayer = imagePlayers_[streamId.getTypeId()][streamId.getInstanceId()].get();
      break;
    case vrs::RecordableTypeId::SlamImuData:
    case vrs::RecordableTypeId::SlamMagnetometerData:
      createMotionPlayer(streamId);
      streamPlayer = motionPlayers_[streamId.getTypeId()][streamId.getInstanceId()].get();
      break;
    case vrs::RecordableTypeId::WifiBeaconRecordableClass:
      createWifiBeaconPlayer(streamId);
      streamPlayer = wifiBeaconPlayer_.get();
      break;
    case vrs::RecordableTypeId::StereoAudioRecordableClass:
      createAudioPlayer(streamId);
      streamPlayer = audioPlayer_.get();
      break;
    case vrs::RecordableTypeId::BluetoothBeaconRecordableClass:
      createBluetoothBeaconPlayer(streamId);
      streamPlayer = bluetoothBeaconPlayer_.get();
      break;
    case vrs::RecordableTypeId::GpsRecordableClass:
      createGpsPlayer(streamId);
      streamPlayer = gpsPlayer_.get();
      break;
    case vrs::RecordableTypeId::BarometerRecordableClass:
      createBarometerPlayer(streamId);
      streamPlayer = barometerPlayer_.get();
      break;
    case vrs::RecordableTypeId::TimeRecordableClass:
      createTimeSyncPlayer(streamId);
      streamPlayer = timeSyncPlayer_.get();
      break;
    case vrs::RecordableTypeId::PoseRecordableClass:
      createPosePlayer(streamId);
      streamPlayer = posePlayer_.get();
      break;
    default:
      fmt::print(
          "setStreamPlayer: Unexpected stream: {}, {}.\n",
          streamId.getNumericName(),
          streamId.getName());
      break;
  }
  if (streamPlayer != nullptr) {
    reader_.setStreamPlayer(streamId, streamPlayer);
    providerStreamIds_.insert(streamId);
    isFirstConfigRecordRead_[streamId.getTypeId()][streamId.getInstanceId()] = false;
  }
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getSlamLeftCameraPlayer() const {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 1));
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getSlamRightCameraPlayer() const {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 2));
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getRgbCameraPlayer() const {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1));
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getEyeCameraPlayer() const {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::EyeCameraRecordableClass, 1));
}

const AriaMotionSensorPlayer* AriaVrsDataProvider::getImuRightPlayer() const {
  return getMotionSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 1));
}

const AriaMotionSensorPlayer* AriaVrsDataProvider::getImuLeftPlayer() const {
  return getMotionSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 2));
}

const AriaMotionSensorPlayer* AriaVrsDataProvider::getMagnetometerPlayer() const {
  return getMotionSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamMagnetometerData, 1));
}

const AriaWifiBeaconPlayer* AriaVrsDataProvider::getWifiBeaconPlayer() const {
  return wifiBeaconPlayer_.get();
}

const AriaAudioPlayer* AriaVrsDataProvider::getAudioPlayer() const {
  return audioPlayer_.get();
}

const AriaBluetoothBeaconPlayer* AriaVrsDataProvider::getBluetoothBeaconPlayer() const {
  return bluetoothBeaconPlayer_.get();
}

const AriaGpsPlayer* AriaVrsDataProvider::getGpsPlayer() const {
  return gpsPlayer_.get();
}

const AriaBarometerPlayer* AriaVrsDataProvider::getBarometerPlayer() const {
  return barometerPlayer_.get();
}

const AriaTimeSyncPlayer* AriaVrsDataProvider::getTimeSyncPlayer() const {
  return timeSyncPlayer_.get();
}

const AriaPosePlayer* AriaVrsDataProvider::getPosePlayer() const {
  return posePlayer_.get();
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getImageSensorPlayer(
    const vrs::StreamId& streamId) const {
  const auto& recordableTypeId = streamId.getTypeId();
  const auto& instanceId = streamId.getInstanceId();
  if (imagePlayers_.find(recordableTypeId) != imagePlayers_.end() &&
      imagePlayers_.at(recordableTypeId).find(instanceId) !=
          imagePlayers_.at(recordableTypeId).end()) {
    return imagePlayers_.at(recordableTypeId).at(instanceId).get();
  }
  return nullptr;
}

const AriaMotionSensorPlayer* AriaVrsDataProvider::getMotionSensorPlayer(
    const vrs::StreamId& streamId) const {
  const auto& recordableTypeId = streamId.getTypeId();
  const auto& instanceId = streamId.getInstanceId();
  if (motionPlayers_.find(recordableTypeId) != motionPlayers_.end() &&
      motionPlayers_.at(recordableTypeId).find(instanceId) !=
          motionPlayers_.at(recordableTypeId).end()) {
    return motionPlayers_.at(recordableTypeId).at(instanceId).get();
  }
  return nullptr;
}

void AriaVrsDataProvider::setVerbose(bool verbose) {
  for (auto& players : imagePlayers_) {
    for (auto& player : players.second) {
      auto& imagePlayer = player.second;
      if (imagePlayer) {
        imagePlayer->setVerbose(verbose);
      }
    }
  }
  for (auto& players : motionPlayers_) {
    for (auto& player : players.second) {
      auto& motionPlayer = player.second;
      if (motionPlayer) {
        motionPlayer->setVerbose(verbose);
      }
    }
  }
  setWifiBeaconPlayerVerbose(verbose);
  setAudioPlayerVerbose(verbose);
  setBluetoothBeaconPlayerVerbose(verbose);
  setGpsPlayerVerbose(verbose);
  setBarometerPlayerVerbose(verbose);
  setTimeSyncPlayerVerbose(verbose);
  setPosePlayerVerbose(verbose);
}

void AriaVrsDataProvider::setImagePlayerVerbose(const vrs::StreamId& streamId, bool verbose) {
  auto& imagePlayer = imagePlayers_.at(streamId.getTypeId()).at(streamId.getInstanceId());
  if (imagePlayer) {
    imagePlayer->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setMotionPlayerVerbose(const vrs::StreamId& streamId, bool verbose) {
  auto& motionPlayer = motionPlayers_.at(streamId.getTypeId()).at(streamId.getInstanceId());
  if (motionPlayer) {
    motionPlayer->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setWifiBeaconPlayerVerbose(bool verbose) {
  if (wifiBeaconPlayer_) {
    wifiBeaconPlayer_->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setAudioPlayerVerbose(bool verbose) {
  if (wifiBeaconPlayer_) {
    wifiBeaconPlayer_->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setBluetoothBeaconPlayerVerbose(bool verbose) {
  if (bluetoothBeaconPlayer_) {
    bluetoothBeaconPlayer_->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setGpsPlayerVerbose(bool verbose) {
  if (gpsPlayer_) {
    gpsPlayer_->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setBarometerPlayerVerbose(bool verbose) {
  if (barometerPlayer_) {
    barometerPlayer_->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setTimeSyncPlayerVerbose(bool verbose) {
  if (timeSyncPlayer_) {
    timeSyncPlayer_->setVerbose(verbose);
  }
}

void AriaVrsDataProvider::setPosePlayerVerbose(bool verbose) {
  if (posePlayer_) {
    posePlayer_->setVerbose(verbose);
  }
}

bool AriaVrsDataProvider::open(
    const std::string& vrsPath,
    const std::string& posePath,
    const std::string& eyetrackingPath,
    const std::string& speechToTextPath) {
  sourcePath_ = vrsPath;
  hasPoses_ = loadPosesFromCsv(posePath);
  hasEyetracks_ = loadEyetrackingFromCsv(eyetrackingPath);
  hasSpeechToText_ = loadSpeechToTextFromCsv(speechToTextPath);
  bool success = openFile(vrsPath);
  // Try loading poses from VRS file
  if (!hasPoses_ &&
      streamExistsInSource(vrs::StreamId(vrs::RecordableTypeId::PoseRecordableClass, 1))) {
    hasLivePoses_ = true;
  }
  return success;
}

double AriaVrsDataProvider::getNextTimestampSec(const vrs::StreamId& streamId) const {
  double nextTimestampSec = -1;
  switch (streamId.getTypeId()) {
    case vrs::RecordableTypeId::SlamCameraData:
    case vrs::RecordableTypeId::RgbCameraRecordableClass:
    case vrs::RecordableTypeId::EyeCameraRecordableClass:
      nextTimestampSec = imagePlayers_.at(streamId.getTypeId())
                             .at(streamId.getInstanceId())
                             ->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::SlamImuData:
    case vrs::RecordableTypeId::SlamMagnetometerData:
      nextTimestampSec = motionPlayers_.at(streamId.getTypeId())
                             .at(streamId.getInstanceId())
                             ->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::WifiBeaconRecordableClass:
      nextTimestampSec = wifiBeaconPlayer_->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::StereoAudioRecordableClass:
      nextTimestampSec = audioPlayer_->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::BluetoothBeaconRecordableClass:
      nextTimestampSec = bluetoothBeaconPlayer_->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::GpsRecordableClass:
      nextTimestampSec = gpsPlayer_->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::BarometerRecordableClass:
      nextTimestampSec = barometerPlayer_->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::TimeRecordableClass:
      nextTimestampSec = timeSyncPlayer_->getNextTimestampSec();
      break;
    case vrs::RecordableTypeId::PoseRecordableClass:
      nextTimestampSec = posePlayer_->getNextTimestampSec();
      break;
    default:
      fmt::print(
          "getNextTimestampSec: Unexpected stream: {}, {}.\n",
          streamId.getNumericName(),
          streamId.getName());
      break;
  }
  return nextTimestampSec;
}

bool AriaVrsDataProvider::tryFetchNextData(
    const vrs::StreamId& streamId,
    double currentTimestampSec) {
  auto nextRecord = getDataRecordByTime(streamId, getNextTimestampSec(streamId));
  if (nextRecord && nextRecord->timestamp < currentTimestampSec) {
    readRecord(*nextRecord);
    return true;
  }
  return false;
}

Eigen::Vector3f AriaVrsDataProvider::getMotionAccelData(const vrs::StreamId& streamId) const {
  const auto motionPlayer = getMotionSensorPlayer(streamId);
  if (motionPlayer) {
    const auto& dataRecord = motionPlayer->getDataRecord();
    return {dataRecord.accelMSec2[0], dataRecord.accelMSec2[1], dataRecord.accelMSec2[2]};
  }
  return {};
}

Eigen::Vector3f AriaVrsDataProvider::getMotionGyroData(const vrs::StreamId& streamId) const {
  const auto motionPlayer = getMotionSensorPlayer(streamId);
  if (motionPlayer) {
    const auto& dataRecord = motionPlayer->getDataRecord();
    return {dataRecord.gyroRadSec[0], dataRecord.gyroRadSec[1], dataRecord.gyroRadSec[2]};
  }
  return {};
}

double AriaVrsDataProvider::getBarometerPressure() const {
  if (barometerPlayer_) {
    const auto& dataRecord = barometerPlayer_->getDataRecord();
    return dataRecord.pressure;
  }
  return -1;
}

double AriaVrsDataProvider::getBarometerTemperature() const {
  if (barometerPlayer_) {
    const auto& dataRecord = barometerPlayer_->getDataRecord();
    return dataRecord.temperature;
  }
  return -1;
}

Eigen::Vector3f AriaVrsDataProvider::getMagnetometerData() const {
  const auto magnetometerPlayer = getMagnetometerPlayer();
  if (magnetometerPlayer) {
    const auto& dataRecord = magnetometerPlayer->getDataRecord();
    return {dataRecord.magTesla[0], dataRecord.magTesla[1], dataRecord.magTesla[2]};
  }
  return {};
}

const std::vector<int32_t>& AriaVrsDataProvider::getAudioData() const {
  if (audioPlayer_) {
    return audioPlayer_->getData().data;
  }
  return {};
}

uint8_t AriaVrsDataProvider::getAudioNumChannels() const {
  if (audioPlayer_) {
    const auto& configRecord = audioPlayer_->getConfigRecord();
    return configRecord.numChannels;
  }
  return 0;
}

const std::vector<uint8_t>& AriaVrsDataProvider::getImageBufferVector(
    const vrs::StreamId& streamId) const {
  const auto imagePlayer = getImageSensorPlayer(streamId);
  if (imagePlayer) {
    return imagePlayer->getData().pixelFrame->getBuffer();
  }
  return {};
}

void* AriaVrsDataProvider::getImageBuffer(const vrs::StreamId& streamId) const {
  const auto imagePlayer = getImageSensorPlayer(streamId);
  if (imagePlayer) {
    return static_cast<void*>(imagePlayer->getData().pixelFrame->getBuffer().data());
  }
  return {};
}

uint32_t AriaVrsDataProvider::getImageWidth(const vrs::StreamId& streamId) const {
  const auto imagePlayer = getImageSensorPlayer(streamId);
  if (imagePlayer) {
    return imagePlayer->getConfigRecord().imageWidth;
  }
  return 0;
}

uint32_t AriaVrsDataProvider::getImageHeight(const vrs::StreamId& streamId) const {
  const auto imagePlayer = getImageSensorPlayer(streamId);
  if (imagePlayer) {
    return imagePlayer->getConfigRecord().imageHeight;
  }
  return 0;
}

double AriaVrsDataProvider::getFastestNominalRateHz() {
  double fastestNominalRateHz = -1;
  for (const auto& streamId : providerStreamIds_) {
    readFirstConfigurationRecord(streamId);
    double nominalRateHz;
    switch (streamId.getTypeId()) {
      case vrs::RecordableTypeId::SlamCameraData:
      case vrs::RecordableTypeId::RgbCameraRecordableClass:
      case vrs::RecordableTypeId::EyeCameraRecordableClass: {
        const auto imagePlayer = getImageSensorPlayer(streamId);
        if (imagePlayer) {
          nominalRateHz = imagePlayer->getConfigRecord().nominalRateHz;
        }
        break;
      }
      case vrs::RecordableTypeId::SlamImuData:
      case vrs::RecordableTypeId::SlamMagnetometerData: {
        const auto motionPlayer = getMotionSensorPlayer(streamId);
        if (motionPlayer) {
          nominalRateHz = motionPlayer->getConfigRecord().nominalRateHz;
        }
        break;
      }
      case vrs::RecordableTypeId::WifiBeaconRecordableClass:
        fmt::print("Sample rate isn't defined for Wifi Beacon stream");
        break;
      case vrs::RecordableTypeId::StereoAudioRecordableClass:
        if (audioPlayer_) {
          nominalRateHz = audioPlayer_->getConfigRecord().sampleRate;
        }
        break;
      case vrs::RecordableTypeId::BluetoothBeaconRecordableClass:
        if (bluetoothBeaconPlayer_) {
          nominalRateHz = bluetoothBeaconPlayer_->getConfigRecord().sampleRateHz;
        }
        break;
      case vrs::RecordableTypeId::GpsRecordableClass:
        if (gpsPlayer_) {
          nominalRateHz = gpsPlayer_->getConfigRecord().sampleRateHz;
        }
        break;
      case vrs::RecordableTypeId::BarometerRecordableClass:
        if (barometerPlayer_) {
          nominalRateHz = barometerPlayer_->getConfigRecord().sampleRate;
        }
        break;
      case vrs::RecordableTypeId::TimeRecordableClass:
        if (timeSyncPlayer_) {
          nominalRateHz = timeSyncPlayer_->getConfigRecord().sampleRateHz;
        }
        break;
      default:
        fmt::print(
            "getFastestNominalRateHz: Unexpected stream: {}, {}.\n",
            streamId.getNumericName(),
            streamId.getName());
        break;
    }
    if (nominalRateHz > fastestNominalRateHz) {
      fastestNominalRateHz = nominalRateHz;
    }
  }
  return fastestNominalRateHz;
}

double AriaVrsDataProvider::getFirstTimestampSec() {
  auto firstTimestampSec = std::numeric_limits<double>::max();
  for (const auto& streamId : providerStreamIds_) {
    readFirstConfigurationRecord(streamId);
    auto timestampSec = getFirstDataRecordTimestampSec(streamId);
    if (timestampSec < firstTimestampSec) {
      firstTimestampSec = timestampSec;
    }
  }
  return firstTimestampSec;
}

bool AriaVrsDataProvider::atLastRecords() {
  for (auto& streamId : providerStreamIds_) {
    if (getLastDataRecord(streamId)->timestamp > getNextTimestampSec(streamId)) {
      return false;
    }
  }
  return true;
}

bool AriaVrsDataProvider::loadDeviceModel() {
  for (auto& players : imagePlayers_) {
    for (auto& player : players.second) {
      auto& imagePlayer = player.second;
      if (imagePlayer) {
        deviceModel_ = datatools::sensors::DeviceModel::fromJson(
            imagePlayer->getConfigRecord().factoryCalibration);
        tryCropAndScaleRgbCameraCalibration();
        tryScaleEtCameraCalibration();
        return true;
      }
    }
  }
  for (auto& players : motionPlayers_) {
    for (auto& player : players.second) {
      auto& motionPlayer = player.second;
      if (motionPlayer) {
        deviceModel_ = datatools::sensors::DeviceModel::fromJson(
            motionPlayer->getConfigRecord().factoryCalibration);
        std::cout
            << "Loaded device model using a motion stream player, may result in invalid RGB camera calibration."
            << std::endl;
        return true;
      }
    }
  }
  // Couldn't find a player to load device model from calibration
  return false;
}

bool AriaVrsDataProvider::tryCropAndScaleRgbCameraCalibration() {
  // RGB calibration is always stored for max resolution
  // If needed we rescale it to match the actual used resolution
  const auto rgbCameraPlayer = getRgbCameraPlayer();
  if (rgbCameraPlayer) {
    const vrs::StreamId rgbStream = rgbCameraPlayer->getStreamId();
    return deviceModel_.tryCropAndScaleCameraCalibration(
        "camera-rgb", kAriaNativeRgbResolution, getImageWidth(rgbStream));
  }
  std::cout << "RGB stream player doesn't exist, cannot update camera calibration" << std::endl;
  return false;
}

bool AriaVrsDataProvider::tryScaleEtCameraCalibration() {
  // Et calibration is always stored for max resolution
  // If needed we rescale it to match the actual used resolution
  // We have to use a division by 2 since left and right ET images are stitched horizontally
  const auto eyeCameraPlayer = getEyeCameraPlayer();
  if (eyeCameraPlayer) {
    const vrs::StreamId eyeStream = eyeCameraPlayer->getStreamId();
    const bool fixCameraEtLeft = deviceModel_.tryCropAndScaleCameraCalibration(
        "camera-et-left", kAriaNativeEtResolution, getImageWidth(eyeStream) / 2);
    const bool fixCameraEtRight = deviceModel_.tryCropAndScaleCameraCalibration(
        "camera-et-right", kAriaNativeEtResolution, getImageWidth(eyeStream) / 2);
    return fixCameraEtLeft && fixCameraEtRight;
  }
  std::cout << "Eye stream player doesn't exist, cannot update camera calibration" << std::endl;
  return false;
}

std::optional<Sophus::SE3d> AriaVrsDataProvider::getPose() const {
  if (hasPoses_) {
    // Always query using the slam-camera-left timestamp.
    int64_t slamCameraLeftTimestampNs = static_cast<int64_t>(
        1e9 * getNextTimestampSec(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 1)));
    return queryPose(slamCameraLeftTimestampNs, imuLeftPoses_);
  } else if (hasLivePoses_) {
    const auto& dataRecord = posePlayer_->getDataRecord();
    Eigen::Vector3d t(
        dataRecord.T_World_ImuLeft_translation[0],
        dataRecord.T_World_ImuLeft_translation[1],
        dataRecord.T_World_ImuLeft_translation[2]);
    Eigen::Quaterniond q(
        dataRecord.T_World_ImuLeft_quaternion[0],
        dataRecord.T_World_ImuLeft_quaternion[1],
        dataRecord.T_World_ImuLeft_quaternion[2],
        dataRecord.T_World_ImuLeft_quaternion[3]);
    return Sophus::SE3d(Sophus::SO3d(q), t);
  }

  fmt::print(stderr, "No poses are loaded. Please check if a valid trajectory file is provided.");
  return {};
}

std::optional<Sophus::SE3d> AriaVrsDataProvider::getPoseOfStreamAtTimestampNs(
    const vrs::StreamId& streamId,
    const int64_t timestampNs) {
  std::optional<Sophus::SE3d> T_world_imuleft;
  if (hasPoses_) {
    T_world_imuleft = queryPose(timestampNs, imuLeftPoses_);
  } else if (hasLivePoses_) {
    double timestampSec = static_cast<double>(timestampNs) / 1e9;
    tryFetchNextData(vrs::StreamId(vrs::RecordableTypeId::PoseRecordableClass, 1), timestampSec);
    T_world_imuleft = getPose();
  }
  if (!T_world_imuleft) {
    return {};
  }
  Sophus::SE3d T_Device_stream;

  if (!kDeviceNumericIdToLabel.count(streamId.getNumericName())) {
    fmt::print(stderr, "Stream {} not supported", streamId.getName());
    return {};
  }
  const std::string labelName = kDeviceNumericIdToLabel.at(streamId.getNumericName());
  if (labelName.find("cam") != std::string::npos) {
    // Camera Calib
    T_Device_stream = deviceModel_.getCameraCalib(labelName)->T_Device_Camera;
  } else if (labelName.find("imu") != std::string::npos) {
    // IMU Calib
    T_Device_stream = deviceModel_.getImuCalib(labelName)->T_Device_Imu;
  }
  auto T_Device_imuleft = deviceModel_.getImuCalib("imu-left")->T_Device_Imu;
  auto T_imuleft_stream = T_Device_imuleft.inverse() * T_Device_stream;
  auto T_world_stream = T_world_imuleft.value() * T_imuleft_stream;
  return T_world_stream;
}

std::optional<Sophus::SE3d> AriaVrsDataProvider::getLatestPoseOfStream(
    const vrs::StreamId& streamId) {
  if (!hasPoses_ && !hasLivePoses_) {
    return {};
  }

  // get latest timestamp of pose
  int64_t currentTimestampNs = static_cast<int64_t>(1e9 * getNextTimestampSec(streamId));
  return getPoseOfStreamAtTimestampNs(streamId, currentTimestampNs);
}

bool AriaVrsDataProvider::loadPosesFromCsv(const std::string& posePath) {
  std::string trajectoryCsvFile = "";
  if (!posePath.empty()) {
    trajectoryCsvFile = posePath;
  } else {
    auto pathSplitted = strSplit(sourcePath_, '/');
    pathSplitted.pop_back();
    for (auto& subfolder : pathSplitted) {
      trajectoryCsvFile += subfolder + "/";
    }
    trajectoryCsvFile += kTrajectoryPathSuffix;
  }
  std::filesystem::path trajectoryCsvPath(trajectoryCsvFile);
  if (!std::filesystem::exists(trajectoryCsvPath)) {
    std::cout << "No pose file found at " << trajectoryCsvPath << " , not visualizing poses"
              << std::endl;
    return false;
  }
  std::cout << "Loading poses file from " << trajectoryCsvFile << std::endl;
  imuLeftPoses_ = readPosesFromCsvFile(trajectoryCsvFile);
  return true;
}

std::optional<Eigen::Vector2f> AriaVrsDataProvider::getEyetracksOnRgbImage() const {
  if (!hasEyetracks_) {
    return {};
  }

  // Always query using the rgb camera timestamp.
  int64_t rgbTimestampNs = static_cast<int64_t>(
      1e9 * getNextTimestampSec(vrs::StreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1)));
  return queryEyetrack(rgbTimestampNs, eyetracksOnRgbImage_);
}

bool AriaVrsDataProvider::loadEyetrackingFromCsv(const std::string& eyetrackingPath) {
  std::string eyetrackingCsvFile = "";
  if (!eyetrackingPath.empty()) {
    eyetrackingCsvFile = eyetrackingPath;
  } else {
    auto pathSplitted = strSplit(sourcePath_, '/');
    pathSplitted.pop_back();
    for (auto& subfolder : pathSplitted) {
      eyetrackingCsvFile += subfolder + "/";
    }
    eyetrackingCsvFile += kEyetrackingPathSuffix;
  }
  std::filesystem::path eyetrackingCsvPath(eyetrackingCsvFile);
  if (!std::filesystem::exists(eyetrackingCsvPath)) {
    std::cout << "No eyetracking file found at " << eyetrackingCsvPath
              << " , not visualizing eye tracks" << std::endl;
    return false;
  }
  std::cout << "Loading eye tracking file from " << eyetrackingCsvFile << std::endl;
  eyetracksOnRgbImage_ = readEyetrackingFromCsvFile(eyetrackingCsvFile);
  return true;
}

std::optional<SpeechToTextDatum> AriaVrsDataProvider::getSpeechToText() const {
  if (!hasSpeechToText_) {
    return {};
  }

  // Always query using the rgb camera timestamp.
  int64_t rgbTimestampNs = static_cast<int64_t>(
      1e9 * getNextTimestampSec(vrs::StreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1)));
  return querySpeechToText(rgbTimestampNs, speechToText_);
}

bool AriaVrsDataProvider::loadSpeechToTextFromCsv(const std::string& speechToTextPath) {
  std::string speechToTextCsvFile = "";
  if (!speechToTextPath.empty()) {
    speechToTextCsvFile = speechToTextPath;
  } else {
    auto pathSplitted = strSplit(sourcePath_, '/');
    pathSplitted.pop_back();
    for (auto& subfolder : pathSplitted) {
      speechToTextCsvFile += subfolder + "/";
    }
    speechToTextCsvFile += kSpeechToTextPathSuffix;
  }
  std::filesystem::path speechToTextCsvPath(speechToTextCsvFile);
  if (!std::filesystem::exists(speechToTextCsvPath)) {
    std::cout << "No speechToText file found at " << speechToTextCsvPath
              << " , not visualizing speech2text" << std::endl;
    return false;
  }
  std::cout << "Loading speech2text file from " << speechToTextCsvFile << std::endl;
  speechToText_ = readSpeechToTextFromCsvFile(speechToTextCsvFile);
  return true;
}

} // namespace dataprovider
} // namespace datatools
} // namespace ark
