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

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include "AriaViewer.h"
#include "utils.h"

namespace {
const std::vector<vrs::StreamId> kStreamIds = {
    vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 1),
    vrs::StreamId(vrs::RecordableTypeId::SlamCameraData, 2),
    vrs::StreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1),
};
} // namespace

using namespace ark::datatools;

int main(int argc, const char* argv[]) {
  if (argc < 2) {
    fmt::print(stderr, "VRS file path must be provided as the argument, exiting.\n");
    return 0;
  }

  std::unique_ptr<dataprovider::AriaDataProvider> dataProvider =
      std::make_unique<dataprovider::AriaVrsDataProvider>();
  const std::string vrsPath = argv[1];
  // override default pose and eyetracking paths from commandline. If left empty
  // strings they will be automatically set according to the folder layout.
  std::string posePath = "";
  std::string eyetrackingPath = "";
  if (argc == 3) {
    posePath = argv[2];
  }
  if (argc == 4) {
    eyetrackingPath = argv[3];
  }

  if (!dataProvider->open(vrsPath, posePath, eyetrackingPath)) {
    fmt::print(stderr, "Failed to open '{}'.\n", vrsPath);
    return 0;
  }
  // Streams should be set after opening VRS file in AriaVrsDataProvider
  for (auto& streamId : kStreamIds) {
    dataProvider->setStreamPlayer(streamId);
  }

  double fastestNominalRateHz = dataProvider->getFastestNominalRateHz();
  double currentTimestampSec = dataProvider->getFirstTimestampSec();
  // Safe to load device model now for both provider modes, VRS configuration records were read
  dataProvider->loadDeviceModel();

  std::unique_ptr<visualization::AriaViewer> viewer =
      std::make_unique<visualization::AriaViewer>(dataProvider.get(), 1280, 800);

  double waitTimeSec = (1 / fastestNominalRateHz) / 10;
  std::thread readerThread([&dataProvider, &viewer, &currentTimestampSec, &waitTimeSec]() {
    while (!dataProvider->atLastRecords()) {
      if (viewer->isPlaying()) {
        {
          std::unique_lock<std::mutex> dataLock(viewer->getDataMutex());
          for (auto& streamId : kStreamIds) {
            if (dataProvider->tryFetchNextData(streamId, currentTimestampSec)) {
              viewer->setCameraImageChanged(true, streamId);
            }
          }
        }
        currentTimestampSec += waitTimeSec;
        std::this_thread::sleep_for(std::chrono::nanoseconds(
            static_cast<int64_t>(waitTimeSec * 1e9 / viewer->getPlaybackSpeedFactor())));
      }
    }
    std::cout << "Finished reading records" << std::endl;
  });
  viewer->run();
  readerThread.join();
  return 0;
}
