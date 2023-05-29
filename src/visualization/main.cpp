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
#include "AriaStreamIds.h"
#include "AriaViewer.h"
#include "utils.h"

#include <algorithm>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <sophus/types.hpp>

#include "trajectoryReader.h"

using namespace ark::datatools;

namespace {
template <
   class result_t = std::chrono::milliseconds,
   class clock_t = std::chrono::steady_clock,
   class duration_t = std::chrono::milliseconds>
auto since(std::chrono::time_point<clock_t, duration_t> const& start) {
 return std::chrono::duration_cast<result_t>(clock_t::now() - start);
};

using namespace ark::datatools::dataprovider;

const std::vector<vrs::StreamId> kImageStreamIds = {
   kSlamLeftCameraStreamId,
   kSlamRightCameraStreamId,
   kRgbCameraStreamId,
   kEyeCameraStreamId};
const std::vector<vrs::StreamId> kImuStreamIds = {kImuRightStreamId, kImuLeftStreamId};
const std::vector<vrs::StreamId> kDataStreams = {
   kMagnetometerStreamId,
   kBarometerStreamId,
   kAudioStreamId,
   kWifiStreamId,
   kBluetoothStreamId,
   kGpsStreamId};

} // namespace


int main(int argc, const char* argv[]) {
 if (argc < 2) {
   fmt::print(stderr, "VRS file path must be provided as the argument, exiting.\n");
   return 0;
 }

 const std::string vrsPath = argv[1];
 const std::string locationPath = argv[2];   // trajectory

 // get and load trajectory first
 TrajectoryBase loadedPoses = readBaseTrajectory(locationPath);
 std::vector<Eigen::Vector3d> devicePoses;
 {
   // Convert the devicePoses to a format we can display with OpenGL
   devicePoses.reserve(loadedPoses.size());
   for (const auto& pose : loadedPoses) {
     devicePoses.push_back(pose.T_world_device.translation());
   }

   // Configure the Window and pangolin OpenGL Context for 3D drawing
   pangolin::CreateWindowAndBind("TrajectoryViewer", 640, 480);
   glEnable(GL_DEPTH_TEST);

   // Define Projection and initial ModelView matrix
   pangolin::OpenGlRenderState s_cam(
       pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
       pangolin::ModelViewLookAt(2.0, 2.0, 1.0, 0, 0, 0, pangolin::AxisZ));

   pangolin::Renderable tree;
   auto axis_i = std::make_shared<pangolin::Axis>();
   axis_i->T_pc = pangolin::OpenGlMatrix::Translate(0., 0., 0);
   tree.Add(axis_i);

   // Create Interactive View in window
   pangolin::SceneHandler handler(tree, s_cam);
   pangolin::View& d_cam = pangolin::CreateDisplay()
                               .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                               .SetHandler(&handler);

   if(0){
     int i = 0;

     std::vector<Eigen::Vector3d> inc_devicePoses;
     inc_devicePoses.reserve(loadedPoses.size());

     while (!pangolin::ShouldQuit() || i < loadedPoses.size()) {
       // Clear screen and activate view to render into
       glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

       {
         inc_devicePoses.push_back(devicePoses[i]);
         d_cam.SetDrawFunction([&](pangolin::View& view) {
           view.Activate(s_cam);
           tree.Render();

           // Display device trajectory
           glColor3f(1, 0.8, 0);
           glLineWidth(3);

           pangolin::glDrawLineStrip(inc_devicePoses);
         });
         i++;
       }
       // Swap frames and Process Events
       pangolin::FinishFrame();
     }
   }


   d_cam.SetDrawFunction([&](pangolin::View& view) {
     view.Activate(s_cam);
     tree.Render();

     // Display device trajectory
     glColor3f(1, 0.8, 0);
     glLineWidth(3);
     pangolin::glDrawLineStrip(devicePoses);
   });

   while (!pangolin::ShouldQuit()) {
     // Clear screen and activate view to render into
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
     // Swap frames and Process Events
     pangolin::FinishFrame();
   }

 }

 // get and open data provider
 std::shared_ptr<dataprovider::AriaDataProvider> dataProvider =
     std::make_shared<dataprovider::AriaVrsDataProvider>();
 // posePath, eyetrackingPath, speechToTextPath
 if (!dataProvider->open(vrsPath)) {
   fmt::print(stderr, "Failed to open '{}'.\n", vrsPath);
   return 0;
 }
 fmt::print(stdout, "Opened '{}'.\n", vrsPath);
 // start viewer with dataprovider
 std::shared_ptr<visualization::AriaViewer> viewer =
     std::make_shared<visualization::AriaViewer>(dataProvider.get(), 1280, 800);
 // initialize and setup datastreams
 auto initDataStreams = viewer->initDataStreams(kImageStreamIds, kImuStreamIds, kDataStreams);
 double currentTimestampSec = initDataStreams.first;
 double fastestNominalRateHz = initDataStreams.second;
 // read and visualize datastreams at desired speed
 const double waitTimeSec = (1. / fastestNominalRateHz) / 10.;
 std::thread readerThread([&viewer, &dataProvider, &currentTimestampSec, &waitTimeSec]() {
   auto start = std::chrono::steady_clock::now();
   while (!dataProvider->atLastRecords()) {
     if (viewer->readData(currentTimestampSec)) {
       currentTimestampSec += waitTimeSec;
       // subtract time it took to load data from wait time
       double thisWaitTimeSec =
           waitTimeSec - since<std::chrono::microseconds>(start).count() * 1e-6;
       if (thisWaitTimeSec > 0.) {
         std::this_thread::sleep_for(std::chrono::nanoseconds(
             static_cast<int64_t>(thisWaitTimeSec * 1e9 / viewer->getPlaybackSpeedFactor())));
       }
       start = std::chrono::steady_clock::now();
     }
   }
   std::cout << "Finished reading records" << std::endl;
 });



 viewer->run();
 readerThread.join();
 return 0;
}