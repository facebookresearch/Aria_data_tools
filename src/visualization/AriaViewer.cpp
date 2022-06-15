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
#include <pangolin/display/image_view.h>
#include <pangolin/gl/glpixformat.h>

// font defined in pangolin
extern const unsigned char AnonymousPro_ttf[];

namespace {
const vrs::StreamId kRgbCameraStreamId(vrs::RecordableTypeId::RgbCameraRecordableClass, 1);
const vrs::StreamId kSlamLeftCameraStreamId(vrs::RecordableTypeId::SlamCameraData, 1);
const vrs::StreamId kSlamRightCameraStreamId(vrs::RecordableTypeId::SlamCameraData, 2);
const vrs::StreamId kImuRightStreamId(vrs::RecordableTypeId::SlamImuData, 1);
const vrs::StreamId kImuLeftStreamId(vrs::RecordableTypeId::SlamImuData, 2);
const vrs::StreamId kMagnetometerStreamId(vrs::RecordableTypeId::SlamMagnetometerData, 1);
const vrs::StreamId kBarometerStreamId(vrs::RecordableTypeId::BarometerRecordableClass, 1);
const vrs::StreamId kAudioStreamId(vrs::RecordableTypeId::StereoAudioRecordableClass, 1);
const vrs::StreamId kWifiStreamId(vrs::RecordableTypeId::WifiBeaconRecordableClass, 1);
const vrs::StreamId kBluetoothStreamId(vrs::RecordableTypeId::BluetoothBeaconRecordableClass, 1);
const vrs::StreamId kGpsStreamId(vrs::RecordableTypeId::GpsRecordableClass, 1);

const std::vector<vrs::StreamId> kImageStreamIds = {
    kSlamLeftCameraStreamId,
    kSlamRightCameraStreamId,
    kRgbCameraStreamId};
const std::vector<vrs::StreamId> kImuStreamIds = {
    vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 1),
    vrs::StreamId(vrs::RecordableTypeId::SlamImuData, 2),
};
const std::vector<vrs::StreamId> kDataStreams = {
    kMagnetometerStreamId,
    kBarometerStreamId,
    kAudioStreamId,
    kWifiStreamId,
    kBluetoothStreamId,
    kGpsStreamId};
pangolin::GlFont kGlFontSpeechToText(AnonymousPro_ttf, 24);
} // namespace

namespace ark {
namespace datatools {
namespace visualization {

AriaViewer::AriaViewer(
    datatools::dataprovider::AriaDataProvider* dataProvider,
    int width,
    int height,
    const std::string& name,
    int id)
    : dataProvider_(dataProvider),
      width_(width),
      height_(height),
      name_(name + std::to_string(id)),
      id_(id) {
  if (!dataProvider->hasPoses()) {
    fmt::print("Not visualizing poses\n");
  }
}

void AriaViewer::run() {
  std::cout << "Start " << name_ << "!" << std::endl;
  // get a static render mutex across all AriaViewer windows
  static std::unique_ptr<std::mutex> render_mutex(new std::mutex());
  std::mutex* p_render_mutex = render_mutex.get();

  pangolin::CreateWindowAndBind(name_, width_, height_);
  pangolin::OpenGlRenderState Visualization3D_camera(
      pangolin::ProjectionMatrix(
          width_ / 2, height_ / 2, width_ / 2, height_ / 2, width_ / 4, height_ / 4, 0.1, 100),
      pangolin::ModelViewLookAt(2.0, 2.0, 1.0, 0, 0, 0, pangolin::AxisZ));
  auto* handler = new pangolin::Handler3D(Visualization3D_camera);
  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  setDataChanged(false, kSlamLeftCameraStreamId);
  setDataChanged(false, kSlamRightCameraStreamId);
  setDataChanged(false, kRgbCameraStreamId);

  auto slamLeftCameraImageWidth = dataProvider_->getImageWidth(kSlamLeftCameraStreamId);
  auto slamLeftCameraImageHeight = dataProvider_->getImageHeight(kSlamLeftCameraStreamId);
  auto slamRightCameraImageWidth = dataProvider_->getImageWidth(kSlamRightCameraStreamId);
  auto slamRightCameraImageHeight = dataProvider_->getImageHeight(kSlamRightCameraStreamId);
  auto rgbCameraImageWidth = dataProvider_->getImageWidth(kRgbCameraStreamId);
  auto rgbCameraImageHeight = dataProvider_->getImageHeight(kRgbCameraStreamId);

  pangolin::ImageView cameraSlamLeftView = pangolin::ImageView("slam left");
  pangolin::ImageView cameraSlamRightView = pangolin::ImageView("slam right");
  // no tag for this one since we want to render text-to-speech
  pangolin::ImageView cameraRgbView = pangolin::ImageView();

  pangolin::View& camTraj = pangolin::Display("camTrajectory").SetAspect(640 / (float)640);
  camTraj.SetHandler(handler);

  // setup a loggers to show the imu signals
  pangolin::DataLog logAcc;
  logAcc.SetLabels(
      {"accRight_x [m/s2]",
       "accRight_y [m/s2]",
       "accRight_z [m/s2]",
       "accLeft_x [m/s2]",
       "accLeft_y [m/s2]",
       "accLeft_z [m/s2]"});
  pangolin::Plotter plotAcc(&logAcc, 0.0f, 1500.f, -20., 20., 100, 5.);
  plotAcc.Track("$i");
  pangolin::DataLog logGyro;
  logGyro.SetLabels(
      {"gyroRight_x [rad/s]",
       "gyroRight_y [rad/s]",
       "gyroRight_z [rad/s]",
       "gyroLeft_x [rad/s]",
       "gyroLeft_y [rad/s]",
       "gyroLeft_z [rad/s]"});
  pangolin::Plotter plotGyro(&logGyro, 0.0f, 1500.f, -3.14, 3.14, 100, 1.);
  plotGyro.Track("$i");
  // magnetometer
  pangolin::DataLog logMag;
  logMag.SetLabels({"mag_x [Tesla]", "mag_y [Tesla]", "mag_z [Tesla]"});
  pangolin::Plotter plotMag(&logMag, 0.0f, 75.f, -100., 100., 100, 1.);
  plotMag.Track("$i");
  // barometer and temperature logs
  pangolin::DataLog logBaro;
  logBaro.SetLabels({"barometer [Pa]"});
  pangolin::Plotter plotBaro(&logBaro, 0.0f, 75.f, 101320 - 20000, 101320 + 1000, 100, 1.);
  plotBaro.Track("$i");
  pangolin::DataLog logTemp;
  logTemp.SetLabels({"temperature [C]"});
  pangolin::Plotter plotTemp(&logTemp, 0.0f, 75.f, 10., 35.0, 100, 1.);
  plotBaro.Track("$i");
  // setup a logger to show the audio signals
  pangolin::DataLog logAudio;
  logAudio.SetLabels({"m0", "m1", "m2", "m3", "m4", "m5", "m6"});
  pangolin::Plotter plotAudio(&logAudio, 0.0f, 3 * 48000.f, -5e-2, 5e-2, 10000, 1e-3f);
  plotAudio.Track("$i");

  auto& container = pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0)
                        .SetLayout(pangolin::LayoutEqual)
                        .AddDisplay(cameraSlamLeftView)
                        .AddDisplay(cameraRgbView)
                        .AddDisplay(cameraSlamRightView)
                        .AddDisplay(camTraj)
                        .AddDisplay(plotAcc)
                        .AddDisplay(plotGyro)
                        .AddDisplay(plotMag)
                        .AddDisplay(plotAudio)
                        .AddDisplay(plotBaro)
                        .AddDisplay(plotTemp);

  // prefix to give each viewer its own set of controls (otherwise they are
  // shared if multiple viewers are opened)
  const std::string prefix = "ui" + std::to_string(id_);
  pangolin::CreatePanel(prefix).SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));
  // Settings
  pangolin::Var<bool> playButton(prefix + ".Play", isPlaying_, true);
  pangolin::Var<float> playbackSlide(
      prefix + ".playback_speed", playbackSpeedFactor_, 0.1, 10, false);
  pangolin::Var<int> sparsitySlide(prefix + ".camSparsity", 1, 1, 10, false);
  pangolin::Var<std::function<void(void)>> save_window(prefix + ".Snapshot UI", [&container]() {
    pangolin::SaveWindowOnRender("snapshot", container.v);
  });
  pangolin::Var<bool> showLeftCamImg(prefix + ".LeftImg", true, true);
  pangolin::Var<bool> showRightCamImg(prefix + ".RightImg", true, true);
  pangolin::Var<bool> showRgbCamImg(prefix + ".RgbImg", true, true);
  pangolin::Var<bool> showLeftCam3D(prefix + ".LeftCam", true, true);
  pangolin::Var<bool> showRightCam3D(prefix + ".RightCam", true, true);
  pangolin::Var<bool> showRgbCam3D(prefix + ".RgbCam", true, true);
  pangolin::Var<bool> showRig3D(prefix + ".Rig", true, true);
  pangolin::Var<bool> showTraj(prefix + ".Trajectory", true, true);
  pangolin::Var<bool> showWorldCoordinateSystem(prefix + ".World Coord.", true, true);
  pangolin::Var<bool> showLeftImu(prefix + ".LeftImu", true, true);
  pangolin::Var<bool> showRightImu(prefix + ".RightImu", true, true);
  pangolin::Var<bool> showMagnetometer(prefix + ".Magnetometer", true, true);
  //  barometer and temperature are pretty constant so we dont show by default
  pangolin::Var<bool> showBarometer(prefix + ".Barometer", false, true);
  pangolin::Var<bool> showTemperature(prefix + ".Temperature", false, true);
  pangolin::Var<bool> showAudio(prefix + ".Audio", true, true);
  // print gps, wifi, bluetooth to terminal output.
  pangolin::Var<bool> printGps(prefix + ".print GPS log", true, true);
  pangolin::Var<bool> printWifi(prefix + ".print Wifi log", true, true);
  pangolin::Var<bool> printBluetooth(prefix + ".print Bluetooth log", true, true);
  // temperature and pressure display on the side of the menue
  pangolin::Var<float> temperatureDisplay(prefix + ".temp [C]", 0., 0.0, 0., false);
  pangolin::Var<float> pressureDisplay(prefix + ".pres [kPa]", 0., 0.0, 0., false);
  // keys 0-9 are used to toggle the different views
  std::array<char, 10> show_hide_keys = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
  for (size_t v = 0; v < container.NumChildren() && v < show_hide_keys.size(); v++) {
    pangolin::RegisterKeyPressCallback(
        show_hide_keys[v], [v, &container]() { container[v].ToggleShow(); });
  }

  // enable blending to allow showing the text-to-speech overlay
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  pangolin::GlFont glFontSpeechToText(AnonymousPro_ttf, 24);

  cameraRgbView.extern_draw_function = [this,
                                        rgbCameraImageWidth,
                                        rgbCameraImageHeight,
                                        &glFontSpeechToText](pangolin::View& v) {
    v.ActivatePixelOrthographic();
    v.ActivateAndScissor();
    std::stringstream text;
    if (speechToText_) {
      // show formatted as "text (confidence, duration)"
      text << speechToText_->text << " (" << std::fixed << std::setprecision(0)
           << 100 * speechToText_->confidence << "%, " << std::fixed << std::setw(5)
           << std::setprecision(3) << speechToText_->duration_s() << "s)";
    }
    glFontSpeechToText.Text(text.str()).DrawWindow(v.v.l, v.v.t() - glFontSpeechToText.Height());

    glLineWidth(3);
    glColor3f(1.0, 0.0, 0.0);
    if (eyetracksOnRgbImage_.size()) {
      // scale to the current display size
      const float scale = (float)v.v.w / (float)rgbCameraImageWidth;
      pangolin::glDrawCross(
          scale * eyetracksOnRgbImage_.back()(0),
          scale * (rgbCameraImageHeight - eyetracksOnRgbImage_.back()(1)),
          10);
    }
    v.GetBounds().DisableScissor();
  };

  // Main loop
  while (!pangolin::ShouldQuit()) {
    isPlaying_ = playButton;
    playbackSpeedFactor_ = playbackSlide;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    {
      const std::lock_guard<std::mutex> dataLock(dataMutex_);
      // Draw images.
      if (cameraSlamLeftView.IsShown() && isDataChanged(kSlamLeftCameraStreamId)) {
        cameraSlamLeftView.SetImage(
            static_cast<void*>(cameraImageBufferMap_[kSlamLeftCameraStreamId.getTypeId()]
                                                    [kSlamLeftCameraStreamId.getInstanceId()]
                                                        .data()),
            slamLeftCameraImageWidth,
            slamLeftCameraImageHeight,
            slamLeftCameraImageWidth,
            pangolin::PixelFormatFromString("GRAY8"));
        setPose(dataProvider_->getPose());
        setDataChanged(false, kSlamLeftCameraStreamId);
      }
      if (cameraRgbView.IsShown() && isDataChanged(kRgbCameraStreamId)) {
        cameraRgbView.SetImage(
            static_cast<void*>(cameraImageBufferMap_[kRgbCameraStreamId.getTypeId()]
                                                    [kRgbCameraStreamId.getInstanceId()]
                                                        .data()),
            rgbCameraImageWidth,
            rgbCameraImageHeight,
            rgbCameraImageWidth * 3,
            pangolin::PixelFormatFromString("RGB24"));
        setEyetracksOnRgbImage(dataProvider_->getEyetracksOnRgbImage());
        setSpeechToText(dataProvider_->getSpeechToText());
        setDataChanged(false, kRgbCameraStreamId);
      }
      if (cameraSlamRightView.IsShown() && isDataChanged(kSlamRightCameraStreamId)) {
        cameraSlamRightView.SetImage(
            static_cast<void*>(cameraImageBufferMap_[kSlamRightCameraStreamId.getTypeId()]
                                                    [kSlamRightCameraStreamId.getInstanceId()]
                                                        .data()),
            slamRightCameraImageWidth,
            slamRightCameraImageHeight,
            slamRightCameraImageWidth,
            pangolin::PixelFormatFromString("GRAY8"));
        setDataChanged(false, kSlamRightCameraStreamId);
      }
    }

    dataProvider_->setWifiBeaconPlayerVerbose(printWifi);
    dataProvider_->setBluetoothBeaconPlayerVerbose(printBluetooth);
    dataProvider_->setGpsPlayerVerbose(printGps);

    if (isDataChanged(kImuRightStreamId) && isDataChanged(kImuLeftStreamId)) {
      setDataChanged(false, kImuRightStreamId);
      setDataChanged(false, kImuLeftStreamId);
      auto& accRightMSec2 =
          accMSec2Map_[kImuRightStreamId.getTypeId()][kImuRightStreamId.getInstanceId()];
      auto& accLeftMSec2 =
          accMSec2Map_[kImuLeftStreamId.getTypeId()][kImuLeftStreamId.getInstanceId()];
      for (size_t i = 0; i < std::min(accLeftMSec2.size(), accRightMSec2.size()); ++i) {
        std::vector<float> acc(6, std::nanf(""));
        if (showRightImu && i < accRightMSec2.size()) {
          acc[0] = accRightMSec2[i](0);
          acc[1] = accRightMSec2[i](1);
          acc[2] = accRightMSec2[i](2);
        }
        if (showLeftImu && i < accLeftMSec2.size()) {
          acc[3] = accLeftMSec2[i](0);
          acc[4] = accLeftMSec2[i](1);
          acc[5] = accLeftMSec2[i](2);
        }
        logAcc.Log(acc);
      }
      auto& gyroRightRadSec =
          gyroRadSecMap_[kImuRightStreamId.getTypeId()][kImuRightStreamId.getInstanceId()];
      auto& gyroLeftRadSec =
          gyroRadSecMap_[kImuLeftStreamId.getTypeId()][kImuLeftStreamId.getInstanceId()];
      for (size_t i = 0; i < std::min(gyroLeftRadSec.size(), gyroRightRadSec.size()); ++i) {
        std::vector<float> gyro(6, std::nanf(""));
        if (showRightImu && i < gyroRightRadSec.size()) {
          gyro[0] = gyroRightRadSec[i](0);
          gyro[1] = gyroRightRadSec[i](1);
          gyro[2] = gyroRightRadSec[i](2);
        }
        if (showLeftImu && i < gyroLeftRadSec.size()) {
          gyro[3] = gyroLeftRadSec[i](0);
          gyro[4] = gyroLeftRadSec[i](1);
          gyro[5] = gyroLeftRadSec[i](2);
        }
        logGyro.Log(gyro);
      }
    }
    if (isDataChanged(kMagnetometerStreamId)) {
      setDataChanged(false, kMagnetometerStreamId);
      for (const auto& mag : magTesla_) {
        logMag.Log(std::vector<float>{mag[0], mag[1], mag[2]});
      }
    }
    if (isDataChanged(kAudioStreamId)) {
      setDataChanged(false, kAudioStreamId);
      for (const auto& audio : audio_) {
        logAudio.Log(audio);
      }
    }
    if (isDataChanged(kBarometerStreamId)) {
      setDataChanged(false, kBarometerStreamId);
      for (const auto& p : pressure_) {
        logBaro.Log(p);
      }
      for (const auto& temp : temperature_) {
        logTemp.Log(temp);
      }
      pressureDisplay = pressure_.back() * 1e-3; // kPa
      temperatureDisplay = temperature_.back(); // C
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
    cameraSlamLeftView.Show(showLeftCamImg);
    cameraSlamRightView.Show(showRightCamImg);
    cameraRgbView.Show(showRgbCamImg);
    plotAudio.Show(showAudio);
    plotAcc.Show(showLeftImu || showRightImu);
    plotGyro.Show(showLeftImu || showRightImu);
    plotBaro.Show(showBarometer);
    plotTemp.Show(showTemperature);
    plotMag.Show(showMagnetometer);

    {
      std::lock_guard<std::mutex> lock(*p_render_mutex);
      pangolin::FinishFrame();
    }
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
  const float sz = 0.03;

  glLineWidth(3);
  int counter = 0;
  while (counter < T_World_ImuLeft_.size()) {
    auto const& T_World_ImuLeft = T_World_ImuLeft_[counter];
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
      auto camSlamLeft = deviceModel_.getCameraCalib("camera-slam-left");
      if (camSlamLeft) {
        auto fs = camSlamLeft->projectionModel.getFocalLengths();
        auto cs = camSlamLeft->projectionModel.getPrincipalPoint();
        // Left cam
        pangolin::glSetFrameOfReference(T_World_CamSlamLeft.matrix());
        pangolin::glDrawFrustum(
            -cs(0) / fs(0), -cs(1) / fs(1), 1. / fs(0), 1. / fs(1), 640, 480, sz * 0.8);
        pangolin::glUnsetFrameOfReference();
      }
    }

    if (showRightCam3D) {
      auto camSlamRight = deviceModel_.getCameraCalib("camera-slam-right");
      if (camSlamRight) {
        auto fs = camSlamRight->projectionModel.getFocalLengths();
        auto cs = camSlamRight->projectionModel.getPrincipalPoint();
        // Right cam
        pangolin::glSetFrameOfReference(T_World_CamSlamRight.matrix());
        pangolin::glDrawFrustum(
            -cs(0) / fs(0), -cs(1) / fs(1), 1. / fs(0), 1. / fs(1), 640, 480, sz * 0.8);
        pangolin::glUnsetFrameOfReference();
      }
    }

    if (showRgbCam3D) {
      auto camRgb = deviceModel_.getCameraCalib("camera-rgb");
      if (camRgb) {
        auto fs = camRgb->projectionModel.getFocalLengths();
        auto cs = camRgb->projectionModel.getPrincipalPoint();
        // Rgb cam
        pangolin::glSetFrameOfReference(T_World_CamRgb.matrix());
        pangolin::glDrawFrustum(
            -cs(0) / fs(0), -cs(1) / fs(1), 1. / fs(0), 1. / fs(1), 1408, 1408, sz * 0.8);
        pangolin::glUnsetFrameOfReference();
      }
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
  if (isDataChanged(kSlamLeftCameraStreamId)) {
    T_World_ImuLeft_.emplace_back(T_Viewer_World_ * T_World_ImuLeft.value());
  }
}

void AriaViewer::setEyetracksOnRgbImage(const std::optional<Eigen::Vector2f>& eyetrackOnRgbImage) {
  if (!eyetrackOnRgbImage) {
    return;
  }
  eyetracksOnRgbImage_.emplace_back(eyetrackOnRgbImage.value());
}

void AriaViewer::setSpeechToText(
    const std::optional<ark::datatools::dataprovider::SpeechToTextDatum>& speechToText) {
  speechToText_ = speechToText;
}

std::pair<double, double> AriaViewer::initDataStreams() {
  // Streams should be set after opening VRS file in AriaVrsDataProvider
  for (auto& streamId : kImageStreamIds) {
    dataProvider_->setStreamPlayer(streamId);
  }
  double fastestNominalRateHz = dataProvider_->getFastestNominalRateHz();
  double currentTimestampSec = dataProvider_->getFirstTimestampSec();
  // Safe to load device model now for both provider modes, VRS configuration records were read
  dataProvider_->loadDeviceModel();
  // init device model (intrinsic and extrinsic calibration)
  deviceModel_ = dataProvider_->getDeviceModel();
  // init transformation from camera coordinate systems to the pose coordinate system (imuLeft).
  auto T_ImuLeft_Device = deviceModel_.getImuCalib("imu-left")->T_Device_Imu.inverse();
  T_ImuLeft_cameraMap_[kSlamLeftCameraStreamId.getTypeId()]
                      [kSlamLeftCameraStreamId.getInstanceId()] = T_ImuLeft_Device *
      deviceModel_.getCameraCalib("camera-slam-left")->T_Device_Camera;
  T_ImuLeft_cameraMap_[kRgbCameraStreamId.getTypeId()][kRgbCameraStreamId.getInstanceId()] =
      T_ImuLeft_Device * deviceModel_.getCameraCalib("camera-rgb")->T_Device_Camera;
  T_ImuLeft_cameraMap_[kSlamRightCameraStreamId.getTypeId()]
                      [kSlamRightCameraStreamId.getInstanceId()] = T_ImuLeft_Device *
      deviceModel_.getCameraCalib("camera-slam-right")->T_Device_Camera;
  // Setup the other datastreams; this is done after the image streams and
  // getting their fastestNominalRate.
  auto vrsDataProvider =
      dynamic_cast<ark::datatools::dataprovider::AriaVrsDataProvider*>(dataProvider_);
  for (auto& streamId : kImuStreamIds) {
    dataProvider_->setStreamPlayer(streamId);
    if (vrsDataProvider) {
      vrsDataProvider->readFirstConfigurationRecord(streamId);
    }
  }
  for (auto& streamId : kDataStreams) {
    dataProvider_->setStreamPlayer(streamId);
    if (vrsDataProvider) {
      vrsDataProvider->readFirstConfigurationRecord(streamId);
    }
  }
  return {currentTimestampSec, fastestNominalRateHz};
}

bool AriaViewer::readData(double currentTimestampSec) {
  if (!dataProvider_->atLastRecords()) {
    if (isPlaying()) {
      {
        std::unique_lock<std::mutex> dataLock(dataMutex_);
        // Handle image streams
        for (auto& streamId : kImageStreamIds) {
          if (dataProvider_->tryFetchNextData(streamId, currentTimestampSec)) {
            setDataChanged(true, streamId);
            setCameraImageBuffer(dataProvider_->getImageBufferVector(streamId), streamId);
          }
        }
        // Handle left and right imu streams
        for (auto& streamId : kImuStreamIds) {
          std::vector<Eigen::Vector3f> accMSec2, gyroRadSec;
          while (dataProvider_->tryFetchNextData(streamId, currentTimestampSec)) {
            accMSec2.push_back(dataProvider_->getMotionAccelData(streamId));
            gyroRadSec.push_back(dataProvider_->getMotionGyroData(streamId));
          }
          setImuDataChunk(streamId, accMSec2, gyroRadSec);
        }
        // handle magnetometer stream
        std::vector<Eigen::Vector3f> magTesla;
        while (dataProvider_->tryFetchNextData(kMagnetometerStreamId, currentTimestampSec)) {
          auto magnetometerData = dataProvider_->getMagnetometerData();
          magTesla.emplace_back(magnetometerData[0], magnetometerData[1], magnetometerData[2]);
        }
        setMagnetometerChunk(kMagnetometerStreamId, magTesla);
        // handle barometer stream
        std::vector<float> temperature, pressure;
        while (dataProvider_->tryFetchNextData(kBarometerStreamId, currentTimestampSec)) {
          temperature.emplace_back(dataProvider_->getBarometerTemperature());
          pressure.emplace_back(dataProvider_->getBarometerPressure());
        }
        setBarometerChunk(kBarometerStreamId, temperature, pressure);
        // handle audio stream
        std::vector<std::vector<float>> audio;
        while (dataProvider_->tryFetchNextData(kAudioStreamId, currentTimestampSec)) {
          auto audioData = dataProvider_->getAudioData();
          const size_t C = dataProvider_->getAudioNumChannels();
          const auto N = audioData.size() / C;
          assert(audioData.size() % C == 0);
          for (size_t i = 0; i < N; ++i) {
            audio.emplace_back();
            for (size_t c = 0; c < C; ++c) {
              // Audio samples are 32bit; convert to float for visualization
              audio.back().emplace_back(
                  (float)(audioData[i * C + c] / (double)std::numeric_limits<int32_t>::max()));
            }
          }
        }
        setAudioChunk(kAudioStreamId, audio);
        // Make sure we fetch next data for wifi, bluetooth, and gps so that
        // callbacks can printout sensor information to the terminal.
        const std::array<const vrs::StreamId, 3> callbackStreamIds = {
            kWifiStreamId, kBluetoothStreamId, kGpsStreamId};
        for (const auto& streamId : callbackStreamIds) {
          while (dataProvider_->tryFetchNextData(streamId, currentTimestampSec)) {
          }
        }
      }
    }
    return true;
  }
  return false;
}

} // namespace visualization
} // namespace datatools
} // namespace ark
