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

#include <models/DeviceModel.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sophus/se3.hpp>
#include <Eigen/Eigen>

using namespace ark::datatools::sensors;

const char* kTestCalibStr = R"rawJsonDelimiter({
  "CameraCalibrations": [
    {
      "Label": "camera-slam-left",
      "Projection": {
        "Name": "FisheyeRadTanThinPrism",
        "Params": [
          240, 320, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        ]
      },
      "T_Device_Camera": {
        "Translation": [0, 0, 0],
        "UnitQuaternion": [
          1,
          [0, 0, 0]
        ]
      }
    },
    {
      "Label": "camera-et-left",
      "Projection": {
        "Name": "KannalaBrandtK3",
        "Params": [
          552, 552, 320, 240, 0, 0, 0, 0
        ]
      },
      "T_Device_Camera": {
        "Translation": [1, 2, 3],
        "UnitQuaternion": [
          1,
          [0, 0, 0]
        ]
      }
    }
  ],
  "ImuCalibrations": [
    {
      "Accelerometer": {
        "Bias": {
          "Name": "Constant",
          "Offset": [
            0.1, 0.2, 0.3
          ]
        },
        "Model": {
          "Name": "Linear",
          "RectificationMatrix": [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
          ]
        }
      },
      "Gyroscope": {
        "Bias": {
          "Name": "Constant",
          "Offset": [
            -0.3, -0.2, -0.1
          ]
        },
        "Model": {
          "Name": "Linear",
          "RectificationMatrix": [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
          ]
        }
      },
      "Label": "imu-left",
      "T_Device_Imu": {
        "Translation": [-1, -2, -3],
        "UnitQuaternion": [
          1,
          [0, 0, 0]
        ]
      }
    }
  ]
})rawJsonDelimiter";

TEST(DeviceModelTest, ParseJsonAndProjectCamera) {
  DeviceModel model = DeviceModel::fromJson(std::string(kTestCalibStr));
  ASSERT_THAT(model.getCameraLabels(), testing::ElementsAre("camera-et-left", "camera-slam-left"));

  // Test FisheyeRadTanThinPrism projection
  const auto slamLeft = model.getCameraCalib("camera-slam-left").value();
  Eigen::Vector3d p_slamLeft{2.0, 3.0, 1.0};
  Eigen::Vector2d uv_slamLeft = slamLeft.projectionModel.project(p_slamLeft);
  Eigen::Vector2d uv_slamLeft_actual{493.09928578162271, 499.64892867243407};
  EXPECT_TRUE((uv_slamLeft - uv_slamLeft_actual).norm() < 1e-6);

  Eigen::Vector3d p_slamLeft_convertBack = slamLeft.projectionModel.unproject(uv_slamLeft);
  EXPECT_TRUE((p_slamLeft_convertBack - p_slamLeft).norm() < 1e-6);

  // Test KB3 projection
  const auto etLeft = model.getCameraCalib("camera-et-left").value();
  Eigen::Vector3d p_etLeft{-1.0, -2.0, 1.0};
  Eigen::Vector2d uv_etLeft = etLeft.projectionModel.project(p_etLeft);
  Eigen::Vector2d uv_etLeft_actual{36.044133853218739, -327.91173229356252};
  EXPECT_TRUE((uv_etLeft - uv_etLeft_actual).norm() < 1e-6);

  Eigen::Vector3d p_etLeft_convertBack = etLeft.projectionModel.unproject(uv_etLeft);
  EXPECT_TRUE((p_etLeft_convertBack - p_etLeft).norm() < 1e-6);
}

TEST(DeviceModelTest, ParseJsonAndTransformBetweenSensors) {
  DeviceModel model = DeviceModel::fromJson(std::string(kTestCalibStr));

  Eigen::Vector3d p_slamLeft{0.1, 0.2, 0.3};
  Eigen::Vector3d p_imuLeft = model.transform(p_slamLeft, "camera-slam-left", "imu-left");
  Eigen::Vector3d p_imuLeft_actual{1.1, 2.2, 3.3};
  EXPECT_TRUE((p_imuLeft - p_imuLeft_actual).norm() < 1e-6);

  Eigen::Vector3d p_slamLeft_convertBack =
      model.transform(p_imuLeft, "imu-left", "camera-slam-left");
  EXPECT_TRUE((p_slamLeft_convertBack - p_slamLeft).norm() < 1e-6);
}

TEST(DeviceModelTest, ParseJsonAndRectifyImu) {
  DeviceModel model = DeviceModel::fromJson(std::string(kTestCalibStr));

  ASSERT_THAT(model.getImuLabels(), testing::ElementsAre("imu-left"));
  const auto imuLeft = model.getImuCalib("imu-left").value();

  Eigen::Vector3d p_imuLeft{1.0, 2.0, 3.0};
  Eigen::Vector3d p_imuLeft_gyroRectified = imuLeft.gyro.rectify(p_imuLeft);
  Eigen::Vector3d p_imuLeft_gyroRectified_actual{1.3, 2.2, 3.1};
  EXPECT_TRUE((p_imuLeft_gyroRectified - p_imuLeft_gyroRectified_actual).norm() < 1e-6);

  Eigen::Vector3d p_imuLeft_accelRectified = imuLeft.accel.rectify(p_imuLeft);
  Eigen::Vector3d p_imuLeft_accelRectified_actual{0.9, 1.8, 2.7};
  EXPECT_TRUE((p_imuLeft_accelRectified - p_imuLeft_accelRectified_actual).norm() < 1e-6);
}
