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

#include "AriaFolderDataProvider.h"
#include "utils.h"

#include <pangolin/pangolin.h>
#include <filesystem>
#include <iostream>

namespace {
constexpr const char* kMetadataPath = "metadata.jsons";
constexpr const char* kTrajectoryPath = "trajectory.csv";
} // namespace

namespace ark {
namespace datatools {
namespace dataprovider {

int64_t getTimestampFromImagePath(const std::string& imagePath) {
  auto tmpSplit = strSplit(strSplit(strSplit(imagePath, '/').back(), '-').back(), '.');
  std::string tsStr = tmpSplit[0] + "." + tmpSplit[1];
  return static_cast<int64_t>(std::stof(tsStr) * 1000) * 1e6;
}

bool AriaFolderDataProvider::open(const std::string& folderPath, const std::string& posePath) {
  sourcePath_ = folderPath;
  if (sourcePath_.back() == '/') {
    sourcePath_.pop_back();
  }
  for (auto streamId : providerStreamIds_) {
    auto streamFolderPath = sourcePath_ + "/" +
        std::to_string(static_cast<int>(streamId.getTypeId())) + "-" +
        std::to_string(streamId.getInstanceId());
    if (!std::filesystem::exists(streamFolderPath)) {
      std::cout << "Stream folder " << streamFolderPath << " does not exist" << std::endl;
      continue;
    }
    std::unique_ptr<StreamInFolder> streamInFolder;
    switch (streamId.getTypeId()) {
      case vrs::RecordableTypeId::SlamCameraData:
      case vrs::RecordableTypeId::RgbCameraRecordableClass:
      case vrs::RecordableTypeId::EyeCameraRecordableClass:
        streamInFolder = std::make_unique<ImageStreamInFolder>();
        break;
      default:
        std::cout << "Invalid stream ID, not implemented yet!" << std::endl;
        break;
    }
    if (streamInFolder) {
      streamInFolder->folderPath = streamFolderPath;
      getDirContent(streamInFolder->folderPath, streamInFolder->filePaths);
      for (const auto& filePath : streamInFolder->filePaths) {
        auto timestampNs = getTimestampFromImagePath(filePath);
        streamInFolder->timestampsNs.push_back(timestampNs);
        streamInFolder->timestampNsToPath[timestampNs] = filePath;
      }
      streamsInFolder_[streamId.getTypeId()][streamId.getInstanceId()] = std::move(streamInFolder);
    }
  }
  hasPoses_ = loadPosesFromCsv(posePath);
  metadataPath_ = sourcePath_ + "/" + kMetadataPath;
  if (std::filesystem::exists(metadataPath_)) {
    loadDeviceModel();
  } else {
    std::cout << "No metadata file found, not loading device model" << std::endl;
  }
  return true;
}

void AriaFolderDataProvider::setStreamPlayer(const vrs::StreamId& streamId) {
  providerStreamIds_.insert(streamId);
}

bool AriaFolderDataProvider::tryFetchNextData(
    const vrs::StreamId& streamId,
    double currentTimestampSec) {
  auto imageStreamInFolder = getImageStream(streamId);
  if (imageStreamInFolder == nullptr ||
      imageStreamInFolder->currentFileIndex >= imageStreamInFolder->timestampsNs.size()) {
    return false;
  }
  auto currentFileTimestampNs =
      imageStreamInFolder->timestampsNs[imageStreamInFolder->currentFileIndex];
  auto currentFileTimestampSec = static_cast<double>(currentFileTimestampNs) / 1e9;
  if (currentFileTimestampSec < currentTimestampSec) {
    imageStreamInFolder->currentFileIndex++;
    imageStreamInFolder->image = std::make_shared<pangolin::TypedImage>(
        pangolin::LoadImage(imageStreamInFolder->timestampNsToPath.at(currentFileTimestampNs)));
    return true;
  }
  return false;
}

void* AriaFolderDataProvider::getImageBuffer(const vrs::StreamId& streamId) const {
  const auto imageStreamInFolder = getImageStream(streamId);
  if (imageStreamInFolder == nullptr) {
    return nullptr;
  }
  return static_cast<void*>(imageStreamInFolder->image->ptr);
}

double AriaFolderDataProvider::getFirstTimestampSec() {
  auto firstTimestampSec = std::numeric_limits<double>::max();
  for (auto& recordableTypeId : streamsInFolder_) {
    for (auto& instanceId : recordableTypeId.second) {
      auto& stream = instanceId.second;
      double streamFirstTimestampSec = static_cast<double>(stream->timestampsNs[0]) / 1e9;
      if (streamFirstTimestampSec < firstTimestampSec) {
        firstTimestampSec = streamFirstTimestampSec;
      }
    }
  }
  return firstTimestampSec;
}

bool AriaFolderDataProvider::atLastRecords() {
  for (auto& recordableTypeId : streamsInFolder_) {
    for (auto& instanceId : recordableTypeId.second) {
      auto& streamInFolder = instanceId.second;
      if (streamInFolder->currentFileIndex < streamInFolder->timestampsNs.size()) {
        return false;
      }
    }
  }
  return true;
}

bool AriaFolderDataProvider::streamExistsInSource(const vrs::StreamId& streamId) {
  return streamsInFolder_.find(streamId.getTypeId()) != streamsInFolder_.end() &&
      streamsInFolder_.at(streamId.getTypeId()).find(streamId.getInstanceId()) !=
      streamsInFolder_.at(streamId.getTypeId()).end();
}

bool AriaFolderDataProvider::loadDeviceModel() {
  std::ifstream fin(metadataPath_);
  if (!fin) {
    std::cerr << "Meta data file: " << metadataPath_ << " doesn't exist.\nQuit." << std::endl;
    return false;
  }
  std::string firstLine;
  // Get First line.
  std::getline(fin, firstLine);
  fin.close();
  fb_rapidjson::Document doc;
  fb_rapidjson::Document calibSubDoc;
  doc.Parse(firstLine.c_str());
  calibSubDoc.Parse(doc["tags"]["calib_json"].GetString());
  deviceModel_ = datatools::sensors::DeviceModel::fromJson(calibSubDoc);
  return true;
}

std::optional<Sophus::SE3d> AriaFolderDataProvider::getPose() const {
  if (!hasPoses_) {
    return {};
  }
  auto& slamLeftCamera = streamsInFolder_.at(vrs::RecordableTypeId::SlamCameraData).at(1);
  if (slamLeftCamera->timestampsNs.empty()) {
    return {};
  }
  auto currentFileTimestampNs = slamLeftCamera->timestampsNs[slamLeftCamera->currentFileIndex];
  return queryPose(currentFileTimestampNs, imuLeftPoses_);
}

bool AriaFolderDataProvider::loadPosesFromCsv(const std::string& posePath) {
  if (!posePath.empty()) {
    trajectoryCsvPath_ = posePath;
  } else {
    trajectoryCsvPath_ = sourcePath_ + "/" + kTrajectoryPath;
  }
  std::filesystem::path trajectoryCsvFile(trajectoryCsvPath_);
  if (!std::filesystem::exists(trajectoryCsvFile)) {
    std::cout << "No pose file found, not visualizing poses" << std::endl;
    return false;
  }
  imuLeftPoses_ = readPosesFromCsvFile(trajectoryCsvPath_);
  return true;
}

ImageStreamInFolder* AriaFolderDataProvider::getImageStream(const vrs::StreamId& streamId) const {
  switch (streamId.getTypeId()) {
    case vrs::RecordableTypeId::SlamCameraData:
    case vrs::RecordableTypeId::RgbCameraRecordableClass:
    case vrs::RecordableTypeId::EyeCameraRecordableClass:
      break;
    default:
      std::cout << "Invalid stream ID for fetching next data, not implemented yet!" << std::endl;
      return nullptr;
  }
  return static_cast<ImageStreamInFolder*>(
      streamsInFolder_.at(streamId.getTypeId()).at(streamId.getInstanceId()).get());
}
} // namespace dataprovider
} // namespace datatools
} // namespace ark
