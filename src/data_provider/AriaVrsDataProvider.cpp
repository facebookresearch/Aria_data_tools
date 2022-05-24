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

namespace {

void printDataLayout(const vrs::CurrentRecord& r, vrs::DataLayout& datalayout) {
  fmt::print(
      "{:.3f} {} record, {} [{}]\n",
      r.timestamp,
      toString(r.recordType),
      r.streamId.getName(),
      r.streamId.getNumericName());
  datalayout.printLayoutCompact(std::cout, "  ");
}

auto kAudioCallback = [](const vrs::CurrentRecord&, std::vector<int32_t>&, bool) { return true; };
auto kBarometerCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& datalayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, datalayout);
      }
      return true;
    };
auto kBluetoothBeaconCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& datalayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, datalayout);
      }
      return true;
    };
auto kGpsCallback = [](const vrs::CurrentRecord& r, vrs::DataLayout& datalayout, bool verbose) {
  if (verbose) {
    printDataLayout(r, datalayout);
  }
  return true;
};
auto kImageCallback = [](const vrs::CurrentRecord&, std::vector<uint8_t>&, bool) { return true; };
auto kMotionCallback = [](const vrs::CurrentRecord& r, vrs::DataLayout& datalayout, bool verbose) {
  if (verbose) {
    printDataLayout(r, datalayout);
  }
  return true;
};
auto kTimeSyncCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& datalayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, datalayout);
      }
      return true;
    };
auto kWifiBeaconCallback =
    [](const vrs::CurrentRecord& r, vrs::DataLayout& datalayout, bool verbose) {
      if (verbose) {
        printDataLayout(r, datalayout);
      }
      return true;
    };
constexpr const char* kTrajectoryPathSuffix = "location/trajectory.csv";
constexpr const char* kEyetrackingPathSuffix = "eyetracking/et_in_rgb_stream.csv";
const int kAriaNativeRgbResolution = 2880;
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
  return reader_.readRecord(record) == vrs::SUCCESS;
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
    default:
      fmt::print("Unexpected stream: {}, {}.\n", streamId.getNumericName(), streamId.getName());
      break;
  }
  if (streamPlayer != nullptr) {
    reader_.setStreamPlayer(streamId, streamPlayer);
    providerStreamIds_.insert(streamId);
    isFirstConfigRecordRead_[streamId.getTypeId()][streamId.getInstanceId()] = false;
  }
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getSlamLeftCameraPlayer() {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 1));
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getSlamRightCameraPlayer() {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 2));
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getRgbCameraPlayer() {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1));
}

const AriaImageSensorPlayer* AriaVrsDataProvider::getEyeCameraPlayer() {
  return getImageSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::EyeCameraRecordableClass, 1));
}

const AriaMotionSensorPlayer* AriaVrsDataProvider::getImuRightPlayer() {
  return getMotionSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 1));
}

const AriaMotionSensorPlayer* AriaVrsDataProvider::getImuLeftPlayer() {
  return getMotionSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 2));
}

const AriaMotionSensorPlayer* AriaVrsDataProvider::getMagnetometerPlayer() {
  return getMotionSensorPlayer(vrs::StreamId(vrs::RecordableTypeId::SlamMagnetometerData, 1));
}

const AriaWifiBeaconPlayer* AriaVrsDataProvider::getWifiBeaconPlayer() {
  return wifiBeaconPlayer_.get();
}

const AriaAudioPlayer* AriaVrsDataProvider::getAudioPlayer() {
  return audioPlayer_.get();
}

const AriaBluetoothBeaconPlayer* AriaVrsDataProvider::getBluetoothBeaconPlayer() {
  return bluetoothBeaconPlayer_.get();
}

const AriaGpsPlayer* AriaVrsDataProvider::getGpsPlayer() {
  return gpsPlayer_.get();
}

const AriaBarometerPlayer* AriaVrsDataProvider::getBarometerPlayer() {
  return barometerPlayer_.get();
}

const AriaTimeSyncPlayer* AriaVrsDataProvider::getTimeSyncPlayer() {
  return timeSyncPlayer_.get();
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
  if (wifiBeaconPlayer_) {
    wifiBeaconPlayer_->setVerbose(verbose);
  }
  if (audioPlayer_) {
    audioPlayer_->setVerbose(verbose);
  }
  if (bluetoothBeaconPlayer_) {
    bluetoothBeaconPlayer_->setVerbose(verbose);
  }
  if (gpsPlayer_) {
    gpsPlayer_->setVerbose(verbose);
  }
  if (barometerPlayer_) {
    barometerPlayer_->setVerbose(verbose);
  }
  if (timeSyncPlayer_) {
    timeSyncPlayer_->setVerbose(verbose);
  }
}

bool AriaVrsDataProvider::open(
    const std::string& vrsPath,
    const std::string& posePath,
    const std::string& eyetrackingPath) {
  sourcePath_ = vrsPath;
  hasPoses_ = loadPosesFromCsv(posePath);
  hasEyetracks_ = loadEyetrackingFromCsv(eyetrackingPath);
  return openFile(vrsPath);
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
    default:
      fmt::print("Unexpected stream: {}, {}.\n", streamId.getNumericName(), streamId.getName());
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
    const auto imagePlayer = getImageSensorPlayer(streamId);
    double nominalRateHz = imagePlayer->getConfigRecord().nominalRateHz;
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

std::optional<Sophus::SE3d> AriaVrsDataProvider::getPose() const {
  if (!hasPoses_) {
    return {};
  }

  // Always query using the slam-camera-left timestamp.
  uint64_t slamCameraLeftTimestampNs = static_cast<uint64_t>(
      1e9 * getNextTimestampSec(vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 1)));
  return queryPose(slamCameraLeftTimestampNs, imuLeftPoses_);
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
  uint64_t slamCameraLeftTimestampNs = static_cast<uint64_t>(
      1e9 * getNextTimestampSec(vrs::StreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1)));
  return queryEyetrack(slamCameraLeftTimestampNs, eyetracksOnRgbImage_);
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

} // namespace dataprovider
} // namespace datatools
} // namespace ark
