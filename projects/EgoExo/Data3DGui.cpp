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

#include "Data3DGui.h"
#include <string>

constexpr int UI_WIDTH = 400;
constexpr int kMapPanelWidth = 1920;
constexpr int kWindowWidth = kMapPanelWidth + UI_WIDTH;
constexpr int kWindowHeight = 1080;

const std::vector<Eigen::Vector3f> kTrajColors{
    {0.12, 0.47, 0.71},
    {1, 0.5, 0.05},
    {0.17, 0.63, 0.17},
    {0.84, 0.15, 0.16},
    {0.58, 0.4, 0.26},
    {0.55, 0.34, 0.29},
    {0.89, 0.47, 0.76},
    {0.5, 0.5, 0.5},
    {0.74, 0.74, 0.13},
    {0.09, 0.75, 0.81}};

extern const unsigned char AnonymousPro_ttf[];
static pangolin::GlFont kGlFont(AnonymousPro_ttf, 20);

namespace pangolin_helpers {
template <typename T>
inline void glDrawFrustum(const Eigen::Matrix<T, 3, 3>& Kinv, int w, int h, GLfloat scale) {
  pangolin::glDrawFrustum(
      (GLfloat)Kinv(0, 2),
      (GLfloat)Kinv(1, 2),
      (GLfloat)Kinv(0, 0),
      (GLfloat)Kinv(1, 1),
      w,
      h,
      scale);
}

template <typename T>
inline void glDrawFrustum(
    const Eigen::Matrix<T, 3, 3>& Kinv,
    int w,
    int h,
    const Eigen::Matrix<T, 4, 4>& T_wf,
    T scale) {
  pangolin::glSetFrameOfReference(T_wf);
  pangolin_helpers::glDrawFrustum(Kinv, w, h, scale);
  pangolin::glUnsetFrameOfReference();
}

Eigen::MatrixXf getPositionBounds(const Eigen::MatrixXf& points) {
  Eigen::MatrixXf bounds(3, 2);
  bounds.col(0) = points.rowwise().minCoeff().cast<float>();
  bounds.col(1) = points.rowwise().maxCoeff().cast<float>();
  return bounds;
}

void centerViewOnMap(
    pangolin::OpenGlRenderState& glcam,
    const std::vector<Eigen::Vector3f>& points,
    const float focalLength,
    const int windowWidth) {
  // Map input point to an Eigen3X matrix
  using Matrix3X = Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::RowMajor>;
  using MatrixCRef = Eigen::Map<const Matrix3X>;
  MatrixCRef pointsEigen(points[0].data(), 3, points.size());

  const Eigen::Vector3f center = getPositionBounds(pointsEigen).rowwise().mean();
  const Eigen::Vector3f gdir{0.0, 0.0, -9.81};
  Eigen::Vector3f nonParallelVec(1, 0, 0);
  if (std::abs(nonParallelVec.dot(gdir)) > 0.99) {
    nonParallelVec << 0, 0, 1;
  }
  const Eigen::Vector3f perpendicularVec = nonParallelVec.cross(gdir);
  const Eigen::Vector3f viewDir = gdir + 2 * perpendicularVec;
  const Eigen::MatrixXf krFromCenter = pointsEigen.colwise() - center;
  const Eigen::VectorXf krDirectionsFromCenter = krFromCenter.colwise().norm();
  const Eigen::VectorXf distances = krDirectionsFromCenter * focalLength / (windowWidth / 2);
  const float distance = distances.maxCoeff();
  const Eigen::Vector3f eye = center - distance * viewDir;
  glcam.SetModelViewMatrix(pangolin::ModelViewLookAtRDF(
      eye.x(),
      eye.y(),
      eye.z(),
      center.x(),
      center.y(),
      center.z(),
      -gdir.x(),
      -gdir.y(),
      -gdir.z()));
}

} // namespace pangolin_helpers

namespace ego_exo {

Data3DGui::Data3DGui(
    const GlobalPointCloud& cloud_world,
    const std::vector<std::vector<Eigen::Vector3f>>& fullTrajs_world,
    const std::vector<GoProCalibration>& goProCalibs)
    : cloud_(cloud_world), fullTrajsWorld_(fullTrajs_world), goProCalibs_(goProCalibs) {
  pangolin::CreateWindowAndBind("EgoExo - Data Inspector", kWindowWidth, kWindowHeight);
  visualization3dState_ = pangolin::OpenGlRenderState(pangolin::ProjectionMatrixRDF_TopLeft(
      kMapPanelWidth, kWindowHeight, 1000, 1000, kMapPanelWidth / 2, kWindowHeight / 2, 0.1, 1000));
  vis3dState_ = std::make_unique<pangolin::Handler3D>(visualization3dState_);
  mapView_ =
      &pangolin::CreateDisplay()
           .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
           .SetLayout(pangolin::LayoutEqual)
           .SetAspect(-static_cast<float>(kMapPanelWidth) / static_cast<float>(kWindowHeight))
           .SetHandler(vis3dState_.get());

  for (int i = 0; i < fullTrajsWorld_.size(); ++i) {
    uiPlotPerTraj_.emplace_back("ui3d.Show Trajectory_" + std::to_string(i), true, true);
  }

  pangolin::CreatePanel("ui3d").SetBounds(0, 1, 0, pangolin::Attach::Pix(UI_WIDTH));

  // Initialize the Camera to look to the scene
  if (!fullTrajsWorld_.empty()) {
    const float focal = 420 / 4;
    pangolin_helpers::centerViewOnMap(
        visualization3dState_, fullTrajsWorld_[0], focal, kMapPanelWidth);
  }

  glEnable(GL_MULTISAMPLE);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  // Init point cloud:
  updatePointCloud();
}

void Data3DGui::draw() {
  // vis 3d
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  mapView_->Activate(visualization3dState_);

  if (var_FilterByMeasurements_.GuiChanged() || var_MapMinMeasurements_.GuiChanged() ||
      var_FilterByInvDepthStd_.GuiChanged() || var_MapMaxInvDistStd_.GuiChanged() ||
      var_FilterByDepthStd_.GuiChanged() || var_MapMaxDistStd_.GuiChanged() ||
      var_FilterByImageGradient_.GuiChanged() || var_MapMinGradient_.GuiChanged()) {
    updatePointCloud();
  }

  // Point cloud visualization
  if (uiPoints_) {
    std::lock_guard<std::mutex> lock(filteredCloudMutex_);
    glColor4f(0.9, 0.95, 1, 0.5);
    pangolin::glDrawPoints(filteredCloud_);
  }

  // Visualization of Aria Trajectories
  if (uiPlotAllTraj_) {
    for (int i = 0; i < fullTrajsWorld_.size(); i++) {
      if (uiPlotPerTraj_[i]) {
        const auto& traj = fullTrajsWorld_[i];
        const auto& color = kTrajColors[i % kTrajColors.size()];
        glColor4f(color[0], color[1], color[2], uiTrajAlpha_);
        pangolin::glDrawLineStrip(traj);
      }
    }
  }

  // GoPro camera visualization as Camera Frustum
  glColor3f(1, 1, 1);
  for (int i = 0; i < goProCalibs_.size(); i++) {
    const Sophus::SE3d& T_world_gopro = goProCalibs_[i].T_world_gopro;
    const auto pgopro_world = T_world_gopro.translation();
    const float f = goProCalibs_[i].intrinsics[0];
    const float w = goProCalibs_[i].width;
    const float h = goProCalibs_[i].height;
    Eigen::Matrix3d K;
    K << f, 0, w / 2, 0, f, h / 2, 0, 0, 1;
    Eigen::Matrix3d Kinv = K.inverse();
    pangolin_helpers::glDrawFrustum(Kinv, w, h, T_world_gopro.matrix(), uiCameraFrustumSize_.Get());
    if (uiShowCameraLabel_) {
      kGlFont.Text(("GoPro" + std::to_string(i)).c_str())
          .Draw(pgopro_world[0], pgopro_world[1], pgopro_world[2]);
    }
  }
}

void Data3DGui::updatePointCloud() {
  std::lock_guard<std::mutex> lock(filteredCloudMutex_);

  const float minGradientSquared = var_MapMinGradient_ * var_MapMinGradient_;
  filteredCloud_.clear();
  filteredCloud_.reserve(cloud_.size());
  for (const auto& point : cloud_) {
    if (var_FilterByMeasurements_ && point.numObservations < var_MapMinMeasurements_) {
      continue;
    }
    if (var_FilterByInvDepthStd_ && point.inverseDistanceStd > var_MapMaxInvDistStd_) {
      continue;
    }
    if (var_FilterByDepthStd_ && point.distanceStd > var_MapMaxDistStd_) {
      continue;
    }
    if (var_FilterByImageGradient_ && point.normImageGradientSquared < minGradientSquared) {
      continue;
    }
    filteredCloud_.emplace_back(point.position_world.cast<float>());
  }
  filteredCloud_.shrink_to_fit();
}

} // namespace ego_exo
