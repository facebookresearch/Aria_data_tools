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

#include "utils.h"
#include <filesystem>
#include <iostream>
#include <set>
#include "csv.h"

namespace ark {
namespace datatools {
namespace dataprovider {

void getDirContent(const std::string& dirPath, std::vector<std::string>& dirContent) {
  std::set<std::string> pathSet;
  for (auto& p : std::filesystem::directory_iterator(dirPath)) {
    pathSet.insert(p.path());
  }
  std::copy(pathSet.begin(), pathSet.end(), std::back_inserter(dirContent));
}

constexpr const char* kTimeSyncPathSuffix = "synchronization/timestamp_map.csv";

std::string getTimeSyncPath(const std::string& vrsPath) {
  std::string timeSyncCsvPath;
  auto pathSplitted = ark::datatools::dataprovider::strSplit(vrsPath, '/');
  pathSplitted.pop_back();
  for (auto& subfolder : pathSplitted) {
    timeSyncCsvPath += subfolder + "/";
  }
  timeSyncCsvPath += kTimeSyncPathSuffix;
  return timeSyncCsvPath;
}

std::map<int64_t, Sophus::SE3d> readPosesFromCsvFile(
    const std::string& inputPoseCsv,
    const int firstN) {
  std::map<int64_t, Sophus::SE3d> timeToPoseMap;
  if (!inputPoseCsv.empty()) {
    io::CSVReader<8> in(inputPoseCsv);
    in.read_header(
        io::ignore_extra_column,
        "# timestamp_unix_ns",
        "x_t_world_left_imu",
        "y_t_world_left_imu",
        "z_t_world_left_imu",
        "qw_R_world_left_imu",
        "qx_R_world_left_imu",
        "qy_R_world_left_imu",
        "qz_R_world_left_imu");
    int64_t ts;
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    int count = 0;
    while (in.read_row(ts, t(0), t(1), t(2), q.w(), q.x(), q.y(), q.z()) &&
           (firstN < 0 || count++ < firstN)) {
      timeToPoseMap[ts] = Sophus::SE3d(Sophus::SO3d(q), t);
    }
  }
  std::cout << "Loaded " << timeToPoseMap.size() << " poses" << std::endl;
  return timeToPoseMap;
}

std::map<int64_t, Eigen::Vector2f> readEyetrackingFromCsvFile(
    const std::string& inputEyetrackingCsv,
    const int firstN) {
  std::map<int64_t, Eigen::Vector2f> timeToEtMap;
  if (!inputEyetrackingCsv.empty()) {
    io::CSVReader<3> in(inputEyetrackingCsv);
    in.read_header(
        io::ignore_extra_column, "timestamp_unix_ns", "calib_x [pixel]", "calib_y [pixel]");
    int64_t ts;
    Eigen::Vector2f etCalib_im;
    int count = 0;
    while (in.read_row(ts, etCalib_im(0), etCalib_im(1)) && (firstN < 0 || count++ < firstN)) {
      timeToEtMap[ts] = etCalib_im;
    }
  }
  std::cout << "Loaded " << timeToEtMap.size() << " eye tracking points" << std::endl;
  return timeToEtMap;
}

std::map<int64_t, SpeechToTextDatum> readSpeechToTextFromCsvFile(
    const std::string& inputSpeechToTextCsv,
    const int firstN) {
  std::map<int64_t, SpeechToTextDatum> timeToSpeechToTextMap;
  if (!inputSpeechToTextCsv.empty()) {
    io::CSVReader<4, io::trim_chars<' ', '\t'>, io::double_quote_escape<',', '"'>> in(
        inputSpeechToTextCsv);
    in.read_header(io::ignore_extra_column, "startTime_ns", "endTime_ns", "written", "confidence");
    int64_t tStart, tEnd;
    std::string text;
    float confidence;
    int count = 0;
    while (in.read_row(tStart, tEnd, text, confidence) && (firstN < 0 || count++ < firstN)) {
      timeToSpeechToTextMap[tStart] = {
          .tStart_ns = tStart, .tEnd_ns = tEnd, .text = text, .confidence = confidence};
    }
  }
  std::cout << "Loaded " << timeToSpeechToTextMap.size() << " speech2text points" << std::endl;
  return timeToSpeechToTextMap;
}

std::map<int64_t, int64_t> readTimeSyncCsv(const std::string& inputTimeSyncCsv, const int firstN) {
  std::map<int64_t, int64_t> timeSyncToTimeRecording;
  if (!inputTimeSyncCsv.empty()) {
    io::CSVReader<2> in(inputTimeSyncCsv);
    in.read_header(io::ignore_extra_column, "deviceTimestampNs", "syncedTimestampNs");
    uint64_t tRecording, tSync;
    int count = 0;
    while (in.read_row(tRecording, tSync) && (firstN < 0 || count++ < firstN)) {
      timeSyncToTimeRecording[tSync] = tRecording;
    }
  }
  std::cout << "Loaded " << timeSyncToTimeRecording.size() << " sync timestamps " << std::endl;
  return timeSyncToTimeRecording;
}

std::vector<std::string> strSplit(const std::string& s, const char delimiter) {
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delimiter)) {
    result.push_back(item);
  }
  return result;
}

std::optional<Sophus::SE3d> queryPose(
    const int64_t timestamp,
    const std::map<int64_t, Sophus::SE3d>& timestampToPose) {
  if (timestamp < timestampToPose.begin()->first || timestamp > timestampToPose.rbegin()->first) {
    return {};
  }
  if (timestampToPose.find(timestamp) != timestampToPose.end()) {
    return {};
  }
  // Interpolation
  auto laterPosePtr = timestampToPose.lower_bound(timestamp);
  auto earlyPosePtr = std::prev(laterPosePtr);
  int64_t tsEarly = earlyPosePtr->first;
  int64_t tsLater = laterPosePtr->first;
  Sophus::SE3d poseEarly = earlyPosePtr->second;
  Sophus::SE3d poseLater = laterPosePtr->second;

  double interpFactor =
      static_cast<double>(timestamp - tsEarly) / static_cast<double>(tsLater - tsEarly);
  auto interpQ = poseEarly.unit_quaternion().slerp(interpFactor, poseLater.unit_quaternion());
  auto interpT =
      (1.0 - interpFactor) * poseEarly.translation() + interpFactor * poseLater.translation();
  Sophus::SE3d result;
  result.translation() = interpT;
  result.setRotationMatrix(interpQ.toRotationMatrix());
  return result;
}

std::optional<Eigen::Vector2f> queryEyetrack(
    const int64_t timestamp,
    const std::map<int64_t, Eigen::Vector2f>& timestampToEyetrack) {
  if (timestamp < timestampToEyetrack.begin()->first ||
      timestamp > timestampToEyetrack.rbegin()->first) {
    return {};
  }
  if (timestampToEyetrack.find(timestamp) != timestampToEyetrack.end()) {
    return {};
  }
  // Interpolation
  auto laterEyePtr = timestampToEyetrack.lower_bound(timestamp);
  auto earlyEyePtr = std::prev(laterEyePtr);
  int64_t tsEarly = earlyEyePtr->first;
  int64_t tsLater = laterEyePtr->first;
  Eigen::Vector2f eyeEarly = earlyEyePtr->second;
  Eigen::Vector2f eyeLater = laterEyePtr->second;

  double interpFactor =
      static_cast<double>(timestamp - tsEarly) / static_cast<double>(tsLater - tsEarly);
  auto interpEye = (1.0 - interpFactor) * eyeEarly + interpFactor * eyeLater;
  return interpEye;
}

std::optional<SpeechToTextDatum> querySpeechToText(
    const int64_t timestamp,
    const std::map<int64_t, SpeechToTextDatum>& timestampToSpeechToText) {
  if (timestamp < timestampToSpeechToText.begin()->first ||
      timestamp > timestampToSpeechToText.rbegin()->second.tEnd_ns) {
    return {};
  }
  if (timestampToSpeechToText.find(timestamp) != timestampToSpeechToText.end()) {
    return {};
  }
  // Interpolation
  auto laterEyePtr = timestampToSpeechToText.lower_bound(timestamp);
  auto earlyEyePtr = std::prev(laterEyePtr);
  int64_t tsEarlyStart = earlyEyePtr->first;
  int64_t tsLaterStart = laterEyePtr->first;
  int64_t tsEarlyEnd = earlyEyePtr->second.tEnd_ns;
  int64_t tsLaterEnd = laterEyePtr->second.tEnd_ns;

  if (tsEarlyStart <= timestamp && timestamp < tsEarlyEnd) {
    return earlyEyePtr->second;
  } else if (tsLaterStart <= timestamp && timestamp < tsLaterEnd) {
    return laterEyePtr->second;
  }
  return {};
}

} // namespace dataprovider
} // namespace datatools
} // namespace ark
