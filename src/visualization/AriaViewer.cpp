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
      pangolin::ModelViewLookAt(0.2, 0, 0.2, 0, 0, 0, pangolin::AxisNegY));
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
  pangolin::CreateDisplay()
      .SetBounds(0.5, 1.0, pangolin::Attach::Pix(180), 1.0)
      .SetLayout(pangolin::LayoutEqualHorizontal)
      .AddDisplay(cameraSlamLeftVideo)
      .AddDisplay(cameraSlamRightVideo);
  pangolin::CreateDisplay()
      .SetBounds(0.0, 0.5, pangolin::Attach::Pix(180), 1.0)
      .SetLayout(pangolin::LayoutEqualHorizontal)
      .AddDisplay(cameraRgbVideo)
      .AddDisplay(camTraj);
  camTraj.SetHandler(handler);

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
  pangolin::Var<float> playbackSlide("ui.playback_speed", playbackSpeedFactor_, 0.1, 10, false);
  pangolin::Var<int> sparsitySlide("ui.camSparsity", 1, 1, 10, false);

  // Main loop
  while (!pangolin::ShouldQuit()) {
    {
      std::unique_lock<std::mutex> dataLock(dataMutex_);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      camTraj.Activate(Visualization3D_camera);
      isPlaying_ = playButton.Get();
      playbackSpeedFactor_ = playbackSlide.Get();
      // Draw images.
      if (cameraImageChangedMap_[kSlamLeftCameraStreamId.getTypeId()]
                                [kSlamLeftCameraStreamId.getInstanceId()]) {
        texCameraSlamLeft.Upload(
            dataProvider_->getImageBuffer(kSlamLeftCameraStreamId), GL_LUMINANCE, GL_UNSIGNED_BYTE);
        auto queriedPose = dataProvider_->getPose();
        if (queriedPose) {
          setPose(queriedPose.value());
        }
        cameraImageChangedMap_[kSlamLeftCameraStreamId.getTypeId()]
                              [kSlamLeftCameraStreamId.getInstanceId()] = false;
      }
      if (cameraImageChangedMap_[kRgbCameraStreamId.getTypeId()]
                                [kRgbCameraStreamId.getInstanceId()]) {
        texCameraRgb.Upload(
            dataProvider_->getImageBuffer(kRgbCameraStreamId), GL_RGB, GL_UNSIGNED_BYTE);
        cameraImageChangedMap_[kRgbCameraStreamId.getTypeId()][kRgbCameraStreamId.getInstanceId()] =
            false;
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
      if (showTraj) {
        drawTraj();
      }
      drawRigs(showRig3D, showLeftCam3D, showRightCam3D, showRgbCam3D, sparsitySlide.Get());
    }

    if (showLeftCamImg.Get()) {
      cameraSlamLeftVideo.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      texCameraSlamLeft.RenderToViewportFlipY();
    }

    if (showRgbCamImg.Get()) {
      cameraRgbVideo.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      texCameraRgb.RenderToViewportFlipY();
    }

    if (showRightCamImg.Get()) {
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

  glBegin(GL_LINE_STRIP);
  for (auto const& p : T_World_ImuLeft_) {
    glVertex3f(p.translation()[0], p.translation()[1], p.translation()[2]);
  }
  glEnd();
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
    Sophus::Matrix4f matImuLeft = T_World_ImuLeft.matrix().cast<float>();
    Sophus::Matrix4f matCamSlamLeft =
        (T_World_ImuLeft *
         T_ImuLeft_cameraMap_[kSlamLeftCameraStreamId.getTypeId()]
                             [kSlamLeftCameraStreamId.getInstanceId()])
            .matrix()
            .cast<float>();
    Sophus::Matrix4f matCamSlamRight =
        (T_World_ImuLeft *
         T_ImuLeft_cameraMap_[kSlamRightCameraStreamId.getTypeId()]
                             [kSlamRightCameraStreamId.getInstanceId()])
            .matrix()
            .cast<float>();
    Sophus::Matrix4f matCamRgb =
        (T_World_ImuLeft *
         T_ImuLeft_cameraMap_[kRgbCameraStreamId.getTypeId()][kRgbCameraStreamId.getInstanceId()])
            .matrix()
            .cast<float>();

    if (counter == T_World_ImuLeft_.size() - 1)
      glColor3f(0.0, 1.0, 0.0);
    else
      glColor3f(0.0, 0.0, 1.0);

    if (showRig3D) {
      // Rig
      glPushMatrix();
      glMultMatrixf((GLfloat*)matImuLeft.data());
      pangolin::glDrawAxis(sz / 2);
      glPopMatrix();
    }
    if (showLeftCam3D) {
      // Left cam
      glPushMatrix();
      glMultMatrixf((GLfloat*)matCamSlamLeft.data());
      pangolin::glDrawFrustum(-cx / fx, -cy / fy, 1. / fx, 1. / fy, width, height, sz * 0.8);
      glPopMatrix();
    }

    if (showRightCam3D) {
      // Right cam
      glPushMatrix();
      glMultMatrixf((GLfloat*)matCamSlamRight.data());
      pangolin::glDrawFrustum(-cx / fx, -cy / fy, 1. / fx, 1. / fy, width, height, sz * 0.8);
      glPopMatrix();
    }

    if (showRgbCam3D) {
      // Rgb cam
      glPushMatrix();
      glMultMatrixf((GLfloat*)matCamRgb.data());
      pangolin::glDrawFrustum(-cx / fx, -cy / fy, 1. / fx, 1. / fy, width, height, sz);
      glPopMatrix();
    }

    glBegin(GL_LINE_STRIP);
    if (showLeftCam3D)
      glVertex3f(matCamSlamLeft(0, 3), matCamSlamLeft(1, 3), matCamSlamLeft(2, 3));
    if (showRgbCam3D)
      glVertex3f(matCamRgb(0, 3), matCamRgb(1, 3), matCamRgb(2, 3));
    if (showRig3D)
      glVertex3f(matImuLeft(0, 3), matImuLeft(1, 3), matImuLeft(2, 3));
    if (showRightCam3D)
      glVertex3f(matCamSlamRight(0, 3), matCamSlamRight(1, 3), matCamSlamRight(2, 3));
    glEnd();

    // Always draw the latest camera.
    if (counter != T_World_ImuLeft_.size() - 1 &&
        counter + camSparsity >= T_World_ImuLeft_.size()) {
      counter = T_World_ImuLeft_.size() - 1;
    } else {
      counter += camSparsity;
    }
  }
}

void AriaViewer::setPose(const std::optional<Sophus::SE3d>& pose) {
  if (!pose) {
    return;
  }
  if (!hasFirstPose_) {
    // Use first pose as the world frame.
    T_Viewer_World_ = pose.value().inverse();
    hasFirstPose_ = true;
  }

  // Set pose based on slam-left-camera timestamps.
  if (cameraImageChangedMap_[kSlamLeftCameraStreamId.getTypeId()]
                            [kSlamLeftCameraStreamId.getInstanceId()]) {
    T_World_ImuLeft_.emplace_back(T_Viewer_World_ * pose.value());
  }
}
} // namespace visualization
} // namespace datatools
} // namespace ark
