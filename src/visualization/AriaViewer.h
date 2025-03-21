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
#include "utils.h"

namespace ark::datatools::visualization {

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

  std::pair<double, double> initDataStreams(
      const std::vector<vrs::StreamId>& kImageStreamIds,
      const std::vector<vrs::StreamId>& kImuStreamIds = {},
      const std::vector<vrs::StreamId>& kDataStreams = {}) override;
};
} // namespace ark::datatools::visualization
