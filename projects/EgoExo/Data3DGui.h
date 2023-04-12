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

#include "GlobalPointCloud.h"
#include "GoProCalibration.h"

#include <thread>

#include <pangolin/pangolin.h>

namespace ego_exo {

class Data3DGui {
 public:
  Data3DGui(
      const GlobalPointCloud& cloud_world,
      const std::vector<std::vector<Eigen::Vector3f>>& fullTrajs_world,
      const std::vector<GoProCalibration>& goProCalibs);

  // update plot's 3d view
  void draw();

 private:
  // Point cloud data & filtered version
  GlobalPointCloud cloud_;
  std::vector<Eigen::Vector3f> filteredCloud_;
  std::mutex filteredCloudMutex_; // Lock for updating point cloud data

  // Trajectories (vector of trajectory)
  const std::vector<std::vector<Eigen::Vector3f>>& fullTrajsWorld_;

  // Go Pro camera calibration data
  const std::vector<GoProCalibration>& goProCalibs_;

  //
  // Pangolin UI component
  //

  pangolin::Var<std::string> uiDelimiterPtCloud_{
      "ui3d.------- SemiDensePoints --------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiPoints_{"ui3d.Show SemiDense points", true, true};

  // Point cloud filtering UI & control
  const int kMinNumObservations_ = 2;
  pangolin::Var<bool> var_FilterByMeasurements_{
      "ui3d.Filter By MinMeasurements",
      kMinNumObservations_ > 0,
      false,
      true};
  pangolin::Var<int> var_MapMinMeasurements_{
      "ui3d.Map MinMeasurements",
      kMinNumObservations_,
      0,
      20};

  const float kMaxInvDistanceStd_ = 0.0003f;
  pangolin::Var<bool> var_FilterByInvDepthStd_{
      "ui3d.Filter By InvDist StdDev",
      kMaxInvDistanceStd_ > 0,
      false,
      true};
  pangolin::Var<float> var_MapMaxInvDistStd_{
      "ui3d.Map Max InvDist StdDev",
      kMaxInvDistanceStd_,
      1e-4,
      1.0f,
      true};

  const float kMaxDistanceStd_ = 0.15f;
  pangolin::Var<bool> var_FilterByDepthStd_{
      "ui3d.Filter By Dist StdDev",
      kMaxDistanceStd_ > 0,
      false,
      true};
  pangolin::Var<float> var_MapMaxDistStd_{
      "ui3d.Map Max Dist StdDev",
      kMaxDistanceStd_,
      1e-4,
      1.0f,
      true};

  pangolin::Var<bool> var_FilterByImageGradient_{"ui3d.Filter By ImageGradient", false, true};
  pangolin::Var<float> var_MapMinGradient_{"ui3d.Map Min Gradient", 2, 0, 100};

  pangolin::Var<std::string> uiDelimiterGoPro_{
      "ui3d.------- Camera controls --------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<bool> uiShowCameraLabel_{"ui3d.Show GoPro camera label", true, true};
  pangolin::Var<double> uiCameraFrustumSize_{"ui3d.GoPro frustum size(m)", 0.3, 0.1, 1.0};

  pangolin::Var<std::string> uiDelimiterAriaTraj_{
      "ui3d.------- Aria camera trajectories --------",
      "",
      pangolin::META_FLAG_READONLY};
  pangolin::Var<float> uiTrajAlpha_{"ui3d.Trajectory alpha", 0.6, 0, 1};
  pangolin::Var<bool> uiPlotAllTraj_{"ui3d.Show all trajectories", true, true};
  std::vector<pangolin::Var<bool>> uiPlotPerTraj_;

  pangolin::OpenGlRenderState visualization3dState_;
  std::unique_ptr<pangolin::Handler3D> vis3dState_;
  pangolin::View* mapView_;

  // Functions
  void updatePointCloud();
};

} // namespace ego_exo
