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

namespace eyetorch {

struct PinholeProjector {
  // Definitions
  static constexpr int kNumParams = 4;
  static constexpr bool kProjectDifferentiableByParams = true;
  static constexpr bool kUnprojectDifferentiableByParams = true;
  static constexpr int kFocalXIdx = 0;
  static constexpr int kFocalYIdx = 1;
  static constexpr int kPrincipalPointXIdx = 2;
  static constexpr int kPrincipalPointYIdx = 3;

  // Function implementations
  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE Eigen::Matrix<ScalarT, 2, 1> project(
      const Eigen::Matrix<ScalarT, 3, 1>& point,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams) {
    const Eigen::Matrix<ScalarT, 2, 1> fl = calibParams.template head<2>();
    const Eigen::Matrix<ScalarT, 2, 1> pp = calibParams.template tail<2>();

    return point.template head<2>().cwiseProduct(fl) / point(2) + pp;
  }

  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE void computeProjectJacobians(
      const Eigen::Matrix<ScalarT, 3, 1>& point,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams,
      Eigen::Matrix<ScalarT, 2, 3>& jProjectionByPoint,
      Eigen::Matrix<ScalarT, 2, kNumParams>& jProjectionByParams) {
    const Eigen::Matrix<ScalarT, 2, 1> fl = calibParams.template head<2>();

    const ScalarT oneOverZ = ScalarT(1) / point(2);

    const Eigen::Matrix<ScalarT, 2, 1> flOverZ = oneOverZ * fl;

    jProjectionByPoint.template leftCols<2>() = flOverZ.asDiagonal();
    jProjectionByPoint.template rightCols<1>() =
        -flOverZ.cwiseProduct(point.template head<2>()) * oneOverZ;

    jProjectionByParams.template leftCols<2>() = (oneOverZ * point.template head<2>()).asDiagonal();
    jProjectionByParams.template rightCols<2>() = Eigen::Matrix<ScalarT, 2, 2>::Identity();
  }

  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE Eigen::Matrix<ScalarT, 3, 1> unproject(
      const Eigen::Matrix<ScalarT, 2, 1>& pixel,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams) {
    const Eigen::Matrix<ScalarT, 2, 1> fl = calibParams.template head<2>();
    const Eigen::Matrix<ScalarT, 2, 1> pp = calibParams.template tail<2>();

    return (Eigen::Matrix<ScalarT, 3, 1>() << (pixel - pp).cwiseQuotient(fl), ScalarT(1))
        .finished();
  }

  template <typename ScalarT>
  static ARK_HOST_DEVICE_INLINE void computeUnprojectJacobians(
      const Eigen::Matrix<ScalarT, 2, 1>& pixel,
      const Eigen::Matrix<ScalarT, kNumParams, 1>& calibParams,
      Eigen::Matrix<ScalarT, 2, 2>& jRayByPixels,
      Eigen::Matrix<ScalarT, 2, kNumParams>& jRayByParams) {
    const Eigen::Matrix<ScalarT, 2, 1> fl = calibParams.template head<2>();
    const Eigen::Matrix<ScalarT, 2, 1> pp = calibParams.template segment<2>(2);

    const Eigen::Matrix<ScalarT, 2, 1> invFl = fl.cwiseInverse();
    const Eigen::Matrix<ScalarT, 2, 1> invFl2 = invFl.cwiseProduct(invFl);

    jRayByPixels = invFl.asDiagonal();

    jRayByParams.template leftCols<2>() = invFl2.cwiseProduct(pp - pixel).asDiagonal();
    jRayByParams.template rightCols<2>() = (-invFl).asDiagonal();
  }
};

} // namespace eyetorch
