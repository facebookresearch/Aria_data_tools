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
#include <vrs/StreamId.h>
#include <map>
#include <set>
#include "models/DeviceModel.h"

namespace ark {
namespace datatools {
namespace dataprovider {

class AriaDataProvider {
 public:
  AriaDataProvider() = default;
  virtual ~AriaDataProvider() = default;
  virtual bool open(
      const std::string& sourcePath,
      const std::string& posePath = "",
      const std::string& eyetrackingPath = "") = 0;
  virtual void setStreamPlayer(const vrs::StreamId& streamId) = 0;
  virtual bool tryFetchNextData(
      const vrs::StreamId& streamId,
      double currentTimestampSec = std::numeric_limits<double>::max()) = 0;
  virtual void* getImageBuffer(const vrs::StreamId& streamId) const = 0;
  virtual uint32_t getImageWidth(const vrs::StreamId& streamId) const = 0;
  virtual uint32_t getImageHeight(const vrs::StreamId& streamId) const = 0;
  virtual double getFastestNominalRateHz() = 0;
  virtual double getFirstTimestampSec() = 0;
  // aria pose side-loading (from csv file) and time-aligned serving
  virtual std::optional<Sophus::SE3d> getPose() const = 0;
  virtual std::optional<Sophus::SE3d> getLatestPoseOfStream(
      const vrs::StreamId& streamId) const = 0;
  virtual std::optional<Sophus::SE3d> getPoseOfStreamAtTimestampNs(
      const vrs::StreamId& streamId,
      const uint64_t timestampNs) const = 0;
  virtual bool loadPosesFromCsv(const std::string& posePath) = 0;
  // eyetracking data side-loading (from csv file) and time-aligned serving
  virtual std::optional<Eigen::Vector2f> getEyetracksOnRgbImage() const = 0;
  virtual bool loadEyetrackingFromCsv(const std::string& eyetrackingPath) = 0;
  virtual bool atLastRecords() = 0;
  virtual bool loadDeviceModel() = 0;

  const datatools::sensors::DeviceModel& getDeviceModel() const {
    return deviceModel_;
  }

  bool hasPoses() const {
    return hasPoses_;
  }

 protected:
  std::set<vrs::StreamId> providerStreamIds_;
  datatools::sensors::DeviceModel deviceModel_;

  bool hasPoses_ = false;
  bool hasEyetracks_ = false;
  std::string sourcePath_;
  std::map<uint64_t, Sophus::SE3d> imuLeftPoses_;
  std::map<uint64_t, Eigen::Vector2f> eyetracksOnRgbImage_;
};
} // namespace dataprovider
} // namespace datatools
} // namespace ark
