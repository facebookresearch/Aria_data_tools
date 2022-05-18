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

#include <Eigen/Dense>

#include <sensors/camera/projection/KannalaBrandtK3.h>

namespace eyetorch {

struct KannalaBrandt3Projector {
  // Definitions
  static constexpr int kNumParams = sensors::KannalaBrandtK3Projection::kNumParams;
  static constexpr bool kProjectDifferentiableByParams = true;
  static constexpr bool kUnprojectDifferentiableByParams = false;
  static constexpr int kFocalXIdx = sensors::KannalaBrandtK3Projection::kFocalXIdx;
  static constexpr int kFocalYIdx = sensors::KannalaBrandtK3Projection::kFocalYIdx;
  static constexpr int kPrincipalPointXIdx =
      sensors::KannalaBrandtK3Projection::kPrincipalPointColIdx;
  static constexpr int kPrincipalPointYIdx =
      sensors::KannalaBrandtK3Projection::kPrincipalPointRowIdx;

  // Function implementations
  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE Eigen::Matrix<ScalarT, 2, 1> project(
      const Eigen::Matrix<ScalarT, 3, 1>& point,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams) {
    return sensors::KannalaBrandtK3Projection::project(point, calibParams);
  }

  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE void computeProjectJacobians(
      const Eigen::Matrix<ScalarT, 3, 1>& point,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams,
      Eigen::Matrix<ScalarT, 2, 3>& jProjectionByPoint,
      Eigen::Matrix<ScalarT, 2, kNumParams>& jProjectionByParams) {
    sensors::KannalaBrandtK3Projection::project(
        point, calibParams, &jProjectionByPoint, &jProjectionByParams);
  }

  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE Eigen::Matrix<ScalarT, 3, 1> unproject(
      const Eigen::Matrix<ScalarT, 2, 1>& pixel,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams) {
    return sensors::KannalaBrandtK3Projection::unproject(pixel, calibParams);
  }

  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE void computeUnprojectJacobians(
      const Eigen::Matrix<ScalarT, 2, 1>& pixel,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams,
      Eigen::Matrix<ScalarT, 2, 2>& jRayByPixels,
      Eigen::Matrix<ScalarT, 2, kNumParams>& /*jRayByParams*/) {
    sensors::KannalaBrandtK3Projection::unprojectUnitPlane(pixel, calibParams, &jRayByPixels);
  }
};

} // namespace eyetorch
