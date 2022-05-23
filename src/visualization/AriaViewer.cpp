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

#include "AriaViewer.h"

namespace {
const vrs::StreamId kRgbCameraStreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1);
const vrs::StreamId kSlamLeftCameraStreamId(vrs::RecordableTypeId::SlamCameraData, 1);
const vrs::StreamId kSlamRightCameraStreamId(vrs::RecordableTypeId::SlamCameraData, 2);
} // namespace

namespace ark {
namespace datatools {
namespace visualization {

AriaViewer::AriaViewer(
    const datatools::dataprovider::AriaDataProvider* dataProvider,
    int width,
    int height)
    : dataProvider_(dataProvider),
      width_(width),
      height_(height),
      deviceModel_(dataProvider->getDeviceModel()) {
  const std::string imuTag = "imu-left";
  auto T_ImuLeft_Device = deviceModel_.getImuCalib(imuTag)->T_Device_Imu.inverse();
  T_ImuLeft_cameraMap_[kSlamLeftCameraStreamId.getTypeId()]
                      [kSlamLeftCameraStreamId.getInstanceId()] = T_ImuLeft_Device *
      deviceModel_.getCameraCalib("camera-slam-left")->T_Device_Camera;
  T_ImuLeft_cameraMap_[kRgbCameraStreamId.getTypeId()][kRgbCameraStreamId.getInstanceId()] =
      T_ImuLeft_Device * deviceModel_.getCameraCalib("camera-rgb")->T_Device_Camera;
  T_ImuLeft_cameraMap_[kSlamRightCameraStreamId.getTypeId()]
                      [kSlamRightCameraStreamId.getInstanceId()] = T_ImuLeft_Device *
      deviceModel_.getCameraCalib("camera-slam-right")->T_Device_Camera;
  if (!dataProvider->hasPoses()) {
    fmt::print("Not visualizing poses\n");
  }
}

void AriaViewer::run() {
  std::cout << "Start Aria Viewer!" << std::endl;
  pangolin::CreateWindowAndBind("Main", width_, height_);
  pangolin::OpenGlRenderState Visualization3D_camera(
      pangolin::ProjectionMatrix(
          width_ / 2, height_ / 2, width_ / 2, height_ / 2, width_ / 4, height_ / 4, 0.1, 100),
      pangolin::ModelViewLookAt(2.0, 2.0, 1.0, 0, 0, 0, pangolin::AxisZ));
  auto* handler = new pangolin::Handler3D(Visualization3D_camera);
  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  cameraImageChangedMap_[kSlamLeftCameraStreamId.getTypeId()]
                        [kSlamLeftCameraStreamId.getInstanceId()] = false;
  cameraImageChangedMap_[kSlamRightCameraStreamId.getTypeId()]
                        [kSlamRightCameraStreamId.getInstanceId()] = false;
  cameraImageChangedMap_[kRgbCameraStreamId.getTypeId()][kRgbCameraStreamId.getInstanceId()] =
      false;

  auto slamLeftCameraImageWidth = dataProvider_->getImageWidth(kSlamLeftCameraStreamId);
  auto slamLeftCameraImageHeight = dataProvider_->getImageHeight(kSlamLeftCameraStreamId);
  auto slamRightCameraImageWidth = dataProvider_->getImageWidth(kSlamRightCameraStreamId);
  auto slamRightCameraImageHeight = dataProvider_->getImageHeight(kSlamRightCameraStreamId);
  auto rgbCameraImageWidth = dataProvider_->getImageWidth(kRgbCameraStreamId);
  auto rgbCameraImageHeight = dataProvider_->getImageHeight(kRgbCameraStreamId);

  pangolin::View& cameraSlamLeftVideo =
      pangolin::Display("imgLeftCam")
          .SetAspect(slamLeftCameraImageWidth / (float)slamLeftCameraImageHeight);
  pangolin::View& cameraRgbVideo =
      pangolin::Display("imgRgbCam").SetAspect(rgbCameraImageWidth / (float)rgbCameraImageHeight);
  pangolin::View& cameraSlamRightVideo =
      pangolin::Display("imgRightCam")
          .SetAspect(slamRightCameraImageWidth / (float)slamRightCameraImageHeight);

  pangolin::GlTexture texCameraSlamLeft(
      slamLeftCameraImageWidth,
      slamLeftCameraImageHeight,
      GL_LUMINANCE,
      false,
      0,
      GL_LUMINANCE,
      GL_UNSIGNED_BYTE);
  pangolin::GlTexture texCameraRgb(
      rgbCameraImageWidth, rgbCameraImageHeight, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  pangolin::GlTexture texCameraSlamRight(
      slamRightCameraImageWidth,
      slamRightCameraImageHeight,
      GL_LUMINANCE,
      false,
      0,
      GL_LUMINANCE,
      GL_UNSIGNED_BYTE);
  pangolin::View& camTraj = pangolin::Display("camTrajectory").SetAspect(640 / (float)640);
  camTraj.SetHandler(handler);
  auto& container = pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0)
                        .SetLayout(pangolin::LayoutEqual)
                        .AddDisplay(cameraSlamLeftVideo)
                        .AddDisplay(cameraSlamRightVideo)
                        .AddDisplay(cameraRgbVideo)
                        .AddDisplay(camTraj);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));
  // Settings
  pangolin::Var<bool> playButton("ui.Play", false, false);
  pangolin::Var<bool> showLeftCamImg("ui.LeftImg", true, true);
  pangolin::Var<bool> showRightCamImg("ui.RightImg", true, true);
  pangolin::Var<bool> showRgbCamImg("ui.RgbImg", true, true);
  pangolin::Var<bool> showLeftCam3D("ui.LeftCam", true, true);
  pangolin::Var<bool> showRightCam3D("ui.RightCam", true, true);
  pangolin::Var<bool> showRgbCam3D("ui.RgbCam", true, true);
  pangolin::Var<bool> showRig3D("ui.Rig", true, true);
  pangolin::Var<bool> showTraj("ui.Trajectory", true, true);
  pangolin::Var<bool> showWorldCoordinateSystem("ui.World Coord.", true, true);
  pangolin::Var<float> playbackSlide("ui.playback_speed", playbackSpeedFactor_, 0.1, 10, false);
  pangolin::Var<int> sparsitySlide("ui.camSparsity", 1, 1, 10, false);
  pangolin::Var<std::function<void(void)>> save_window(
      "ui.Snapshot UI", [&container]() { pangolin::SaveWindowOnRender("snapshot", container.v); });
  // keys 0-9 are used to toggle the different views
  std::array<char, 10> show_hide_keys = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
  for (size_t v = 0; v < container.NumChildren() && v < show_hide_keys.size(); v++) {
    pangolin::RegisterKeyPressCallback(
        show_hide_keys[v], [v, &container]() { container[v].ToggleShow(); });
  }

  cameraRgbVideo.extern_draw_function =
      [this, rgbCameraImageWidth, rgbCameraImageHeight](pangolin::View& v) {
        v.ActivatePixelOrthographic();
        glColor3f(1.0, 0.0, 0.0);
        glLineWidth(3);
        pangolin::glDrawLineStrip(eyetracksOnRgbImage_);
        if (eyetracksOnRgbImage_.size()) {
          // scale to the current display size
          const float scale = (float)v.v.w / (float)rgbCameraImageWidth;
          pangolin::glDrawCross(
              scale * eyetracksOnRgbImage_.back()(0),
              scale * (rgbCameraImageHeight - eyetracksOnRgbImage_.back()(1)),
              10);
        }
      };

  // Main loop
  while (!pangolin::ShouldQuit()) {
    {
      std::unique_lock<std::mutex> dataLock(dataMutex_);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      isPlaying_ = playButton.Get();
      playbackSpeedFactor_ = playbackSlide.Get();
      // Draw images.
      if (cameraImageChangedMap_[kSlamLeftCameraStreamId.getTypeId()]
                                [kSlamLeftCameraStreamId.getInstanceId()]) {
        texCameraSlamLeft.Upload(
            dataProvider_->getImageBuffer(kSlamLeftCameraStreamId), GL_LUMINANCE, GL_UNSIGNED_BYTE);
        setPose(dataProvider_->getPose());
        cameraImageChangedMap_[kSlamLeftCameraStreamId.getTypeId()]
                              [kSlamLeftCameraStreamId.getInstanceId()] = false;
      }
      if (cameraImageChangedMap_[kRgbCameraStreamId.getTypeId()]
                                [kRgbCameraStreamId.getInstanceId()]) {
        texCameraRgb.Upload(
            dataProvider_->getImageBuffer(kRgbCameraStreamId), GL_RGB, GL_UNSIGNED_BYTE);
        cameraImageChangedMap_[kRgbCameraStreamId.getTypeId()][kRgbCameraStreamId.getInstanceId()] =
            false;
        setEyetracksOnRgbImage(dataProvider_->getEyetracksOnRgbImage());
      }
      if (cameraImageChangedMap_[kSlamRightCameraStreamId.getTypeId()]
                                [kSlamRightCameraStreamId.getInstanceId()]) {
        texCameraSlamRight.Upload(
            dataProvider_->getImageBuffer(kSlamRightCameraStreamId),
            GL_LUMINANCE,
            GL_UNSIGNED_BYTE);
        cameraImageChangedMap_[kSlamRightCameraStreamId.getTypeId()]
                              [kSlamRightCameraStreamId.getInstanceId()] = false;
      }
    }
    if (hasFirstPose_) {
      // draw 3D
      camTraj.Activate(Visualization3D_camera);
      // draw origin of world coordinate system
      if (showWorldCoordinateSystem) {
        glLineWidth(3);
        pangolin::glDrawAxis(0.3);
      }
      if (showTraj) {
        drawTraj();
      }
      drawRigs(showRig3D, showLeftCam3D, showRightCam3D, showRgbCam3D, sparsitySlide.Get());
    }

    // propagate show parameters
    container[0].Show(showLeftCamImg);
    container[1].Show(showRightCamImg);
    container[2].Show(showRgbCamImg);
    cameraSlamLeftVideo.Show(showLeftCamImg);
    cameraSlamRightVideo.Show(showRightCamImg);
    cameraRgbVideo.Show(showRgbCamImg);

    if (cameraSlamLeftVideo.IsShown()) {
      cameraSlamLeftVideo.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      texCameraSlamLeft.RenderToViewportFlipY();
    }

    if (cameraRgbVideo.IsShown()) {
      cameraRgbVideo.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      texCameraRgb.RenderToViewportFlipY();
    }

    if (cameraSlamRightVideo.IsShown()) {
      cameraSlamRightVideo.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      texCameraSlamRight.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();
  }
  std::cout << "\nQuit Viewer." << std::endl;
  exit(1);
}

void AriaViewer::drawTraj() {
  glColor3f(1, 0.8, 0);
  glLineWidth(3);
  std::vector<Eigen::Vector3d> trajectory;
  for (auto const& T_wi : T_World_ImuLeft_) {
    trajectory.emplace_back(T_wi.translation());
  }
  pangolin::glDrawLineStrip(trajectory);
}

void AriaViewer::drawRigs(
    bool showRig3D,
    bool showLeftCam3D,
    bool showRightCam3D,
    bool showRgbCam3D,
    int camSparsity) {
  // TODO: Remove hardcode
  float sz = 0.03;
  float cx = 200;
  float cy = 200;
  float fx = 400;
  float fy = 400;
  int width = 400;
  int height = 400;

  glLineWidth(3);

  int counter = 0;
  while (counter < T_World_ImuLeft_.size()) {
    auto const& T_World_ImuLeft = T_World_ImuLeft_[counter];
    // Sophus::Matrix4f matImuLeft = T_World_ImuLeft.matrix().cast<float>();
    const auto T_World_CamSlamLeft = T_World_ImuLeft *
        T_ImuLeft_cameraMap_[kSlamLeftCameraStreamId.getTypeId()]
                            [kSlamLeftCameraStreamId.getInstanceId()];
    const auto T_World_CamSlamRight = T_World_ImuLeft *
        T_ImuLeft_cameraMap_[kSlamRightCameraStreamId.getTypeId()]
                            [kSlamRightCameraStreamId.getInstanceId()];
    const auto T_World_CamRgb = T_World_ImuLeft *
        T_ImuLeft_cameraMap_[kRgbCameraStreamId.getTypeId()][kRgbCameraStreamId.getInstanceId()];

    if (counter == T_World_ImuLeft_.size() - 1)
      glColor3f(0.0, 1.0, 0.0);
    else
      glColor3f(0.0, 0.0, 1.0);

    if (showRig3D) {
      // Rig
      pangolin::glDrawAxis(T_World_ImuLeft.matrix(), sz / 2);
    }
    if (showLeftCam3D) {
      // Left cam
      pangolin::glSetFrameOfReference(T_World_CamSlamLeft.matrix());
      pangolin::glDrawFrustum(-cx / fx, -cy / fy, 1. / fx, 1. / fy, width, height, sz * 0.8);
      pangolin::glUnsetFrameOfReference();
    }

    if (showRightCam3D) {
      // Right cam
      pangolin::glSetFrameOfReference(T_World_CamSlamRight.matrix());
      pangolin::glDrawFrustum(-cx / fx, -cy / fy, 1. / fx, 1. / fy, width, height, sz * 0.8);
      pangolin::glUnsetFrameOfReference();
    }

    if (showRgbCam3D) {
      // Rgb cam
      pangolin::glSetFrameOfReference(T_World_CamRgb.matrix());
      pangolin::glDrawFrustum(-cx / fx, -cy / fy, 1. / fx, 1. / fy, width, height, sz);
      pangolin::glUnsetFrameOfReference();
    }

    // draw line connecting rig coordinate frames
    pangolin::glDrawLineStrip(std::vector<Eigen::Vector3d>{
        T_World_CamSlamLeft.translation(),
        T_World_CamRgb.translation(),
        T_World_ImuLeft.translation(),
        T_World_CamSlamRight.translation(),
    });

    // Always draw the latest camera.
    if (counter != T_World_ImuLeft_.size() - 1 &&
        counter + camSparsity >= T_World_ImuLeft_.size()) {
      counter = T_World_ImuLeft_.size() - 1;
    } else {
      counter += camSparsity;
    }
  }
}

void AriaViewer::setPose(const std::optional<Sophus::SE3d>& T_World_ImuLeft) {
  if (!T_World_ImuLeft) {
    return;
  }
  if (!hasFirstPose_) {
    // Use first pose translation to define the world frame.
    // Rotation is gravity aligned so we leave it as is.
    T_Viewer_World_ = Sophus::SE3d(Sophus::SO3d(), -T_World_ImuLeft.value().translation());
    hasFirstPose_ = true;
  }

  // Set pose based on slam-left-camera timestamps.
  if (cameraImageChangedMap_[kSlamLeftCameraStreamId.getTypeId()]
                            [kSlamLeftCameraStreamId.getInstanceId()]) {
    T_World_ImuLeft_.emplace_back(T_Viewer_World_ * T_World_ImuLeft.value());
  }
}

void AriaViewer::setEyetracksOnRgbImage(const std::optional<Eigen::Vector2f>& eyetrackOnRgbImage) {
  if (!eyetrackOnRgbImage) {
    return;
  }
  eyetracksOnRgbImage_.emplace_back(eyetrackOnRgbImage.value());
}

} // namespace visualization
} // namespace datatools
} // namespace ark
