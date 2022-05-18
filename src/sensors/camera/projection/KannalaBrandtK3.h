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

#include <cmath>

#include <camera/projection/common.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <Eigen/Core>

namespace ark {
namespace datatools {
namespace sensors {

// Kannala and Brandt Like 'Generic' Projection Model
// http://cs.iupui.edu/~tuceryan/pdf-repository/Kannala2006.pdf
// https://april.eecs.umich.edu/wiki/Camera_suite
// NOTE, our implementation presents some important differences wrt the original paper:
// - k1 in eq(6) in the paper is fixed to 1.0, so k0 here is k2 in the paper
//   (for the same reason we have only x4 k parameters instead of x5 in the paper, for order
//   theta^9)
//
// In this projection model the points behind the camera are projected in a way that the
// optimization cost function is continuous, therefore the optimization problem can be nicely
// solved. This option should be used during calibration.
//
// parameters = fx, fy, cx, cy, kb0, kb1, kb2, kb3
class KannalaBrandtK3Projection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumParams = 8;
  static constexpr char kName[] = "KannalaBrandtK3";
  static constexpr char kDescription[] = "fx, fy, cx, cy, kb0, kb1, kb2, kb3";
  static constexpr int kNumDistortionParams = 4;
  static constexpr int kFocalXIdx = 0;
  static constexpr int kFocalYIdx = 1;
  static constexpr int kPrincipalPointColIdx = 2;
  static constexpr int kPrincipalPointRowIdx = 3;
  static constexpr bool kIsFisheye = true;
  static constexpr bool kHasAnalyticalProjection = true;

  // Takes in 3-point ``pointOptical`` in the local reference frame of the camera and projects it
  // onto the image plan.
  //
  // Precondition: pointOptical.z() != 0.
  //
  // Return 2-point in the image plane.
  //
  template <
      class D,
      class DP,
      class DJ1 = Eigen::Matrix<typename D::Scalar, 2, 3>,
      class DJ2 = Eigen::Matrix<typename D::Scalar, 2, kNumParams>>
  static ARK_HOST_DEVICE Eigen::Matrix<typename D::Scalar, 2, 1> project(
      const Eigen::MatrixBase<D>& pointOptical,
      const Eigen::MatrixBase<DP>& params,
      Eigen::MatrixBase<DJ1>* d_point = nullptr,
      Eigen::MatrixBase<DJ2>* d_params = nullptr) {
    using T = typename D::Scalar;

    static_assert(
        D::RowsAtCompileTime == 3 && D::ColsAtCompileTime == 1,
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");
    static_assert(
        DP::ColsAtCompileTime == 1 &&
            (DP::RowsAtCompileTime == kNumParams || DP::RowsAtCompileTime == Eigen::Dynamic),
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");
    static_assert(
        (DJ1::ColsAtCompileTime == 3 || DJ1::ColsAtCompileTime == Eigen::Dynamic) &&
            (DJ1::RowsAtCompileTime == 2 || DJ1::RowsAtCompileTime == Eigen::Dynamic),
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");
    static_assert(
        (DJ2::ColsAtCompileTime == kNumParams || DJ2::ColsAtCompileTime == Eigen::Dynamic) &&
            (DJ2::RowsAtCompileTime == 2 || DJ2::RowsAtCompileTime == Eigen::Dynamic),
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");

    SOPHUS_ENSURE(pointOptical.z() != T(0), "z(%) must not be zero.", pointOptical.z());

    // Focal length and principal point
    const Eigen::Matrix<T, 2, 1> ff = params.template head<2>();
    const Eigen::Matrix<T, 2, 1> pp = params.template segment<2>(2);

    const T k0 = params[4];
    const T k1 = params[5];
    const T k2 = params[6];
    const T k3 = params[7];

    const T radiusSquared = pointOptical(0) * pointOptical(0) + pointOptical(1) * pointOptical(1);
    using std::atan2;
    using std::sqrt;

    if (radiusSquared > Sophus::Constants<T>::epsilon()) {
      const T radius = sqrt(radiusSquared);
      const T radiusInverse = T(1.0) / radius;
      const T theta = atan2(radius, pointOptical(2));
      const T theta2 = theta * theta;
      const T theta4 = theta2 * theta2;
      const T theta6 = theta4 * theta2;
      const T theta8 = theta4 * theta4;
      const T rDistorted = theta * (T(1.0) + k0 * theta2 + k1 * theta4 + k2 * theta6 + k3 * theta8);
      const T scaling = rDistorted * radiusInverse;

      if (d_point) {
        const T xSquared = pointOptical(0) * pointOptical(0);
        const T ySquared = pointOptical(1) * pointOptical(1);
        const T normSquared = pointOptical(2) * pointOptical(2) + radiusSquared;
        const T rDistortedDerivative = T(1.0) + T(3.0) * k0 * theta2 + T(5.0) * k1 * theta4 +
            T(7.0) * k2 * theta6 + T(9.0) * k3 * theta8;
        const T x13 = pointOptical(2) * rDistortedDerivative / normSquared - scaling;
        const T rDistortedDerivativeNormalized = rDistortedDerivative / normSquared;
        const T x20 =
            pointOptical(2) * rDistortedDerivative / (normSquared)-radiusInverse * rDistorted;

        (*d_point)(0, 0) = xSquared / radiusSquared * x20 + scaling;
        (*d_point)(0, 1) = pointOptical(1) * x13 * pointOptical(0) / radiusSquared;
        (*d_point)(0, 2) = -pointOptical(0) * rDistortedDerivativeNormalized;
        (*d_point)(1, 0) = (*d_point)(0, 1);
        (*d_point)(1, 1) = ySquared / radiusSquared * x20 + scaling;
        (*d_point)(1, 2) = -pointOptical(1) * rDistortedDerivativeNormalized;

        // toDenseMatrix() is needed for CUDA to explicitly know the matrix dimensions
        (*d_point) = ff.asDiagonal().toDenseMatrix() * (*d_point);
      }
      using std::pow;
      if (d_params) {
        const T xScaled = pointOptical[0] * params[0] * radiusInverse;
        const T yScaled = pointOptical[1] * params[1] * radiusInverse;

        const T theta3 = theta * theta2;
        const T theta5 = theta3 * theta2;
        const T theta7 = theta5 * theta2;
        const T theta9 = theta7 * theta2;

        (*d_params)(0, 0) = pointOptical[0] * scaling;
        (*d_params)(0, 1) = T(0.0);
        (*d_params)(0, 2) = T(1.0);
        (*d_params)(0, 3) = T(0.0);
        (*d_params)(0, 4) = xScaled * theta3;
        (*d_params)(0, 5) = xScaled * theta5;
        (*d_params)(0, 6) = xScaled * theta7;
        (*d_params)(0, 7) = xScaled * theta9;
        (*d_params)(1, 0) = T(0.0);
        (*d_params)(1, 1) = pointOptical[1] * scaling;
        (*d_params)(1, 2) = T(0.0);
        (*d_params)(1, 3) = T(1.0);
        (*d_params)(1, 4) = yScaled * theta3;
        (*d_params)(1, 5) = yScaled * theta5;
        (*d_params)(1, 6) = yScaled * theta7;
        (*d_params)(1, 7) = yScaled * theta9;
      }

      const Eigen::Matrix<T, 2, 1> px =
          scaling * ff.cwiseProduct(pointOptical.template head<2>()) + pp;

      return px;
    } else {
      // linearize r around radius=0
      if (d_point) {
        const T z2 = pointOptical(2) * pointOptical(2);
        // clang-format off
        (*d_point) << ff.x() / pointOptical(2), T(0.0), -ff.x() * pointOptical(0) / z2,
                      T(0.0), ff.y() / pointOptical(2), -ff.y() * pointOptical(1) / z2;
        // clang-format on
      }
      if (d_params) {
        (*d_params)(0, 0) = pointOptical(0) / pointOptical(2);
        (*d_params)(0, 1) = T(0.0);
        (*d_params)(0, 2) = T(1.0);
        (*d_params)(0, 3) = T(0.0);

        (*d_params)(1, 0) = T(0.0);
        (*d_params)(1, 1) = pointOptical(1) / pointOptical(2);
        (*d_params)(1, 2) = T(0.0);
        (*d_params)(1, 3) = T(1.0);
        (*d_params).template rightCols<4>().setZero();
      }
      const Eigen::Matrix<T, 2, 1> px =
          ff.cwiseProduct(pointOptical.template head<2>()) / pointOptical(2) + pp;

      return px;
    }
  }

  // Takes in 2-point ``uv`` in the image plane of the camera and unprojects it
  // into the reference frame of the camera.
  //
  // This function is the inverse of ``project``. In particular it holds that
  //
  // X = unproject(project(X))     [for X=(x,y,z) in R^3, z>0]
  //
  //  and
  //
  // x = project(unproject(s*x))   [for s!=0 and x=(u,v) in R^2]
  //
  // Return 3-point in the camera frame with z = 1.
  //
  template <typename D, typename DP>
  static ARK_HOST_DEVICE Eigen::Matrix<typename D::Scalar, 3, 1> unproject(
      const Eigen::MatrixBase<D>& uvPixel,
      const Eigen::MatrixBase<DP>& params) {
    EIGEN_STATIC_ASSERT(
        D::RowsAtCompileTime == 2 && D::ColsAtCompileTime == 1,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(
        DP::ColsAtCompileTime == 1 &&
            (DP::RowsAtCompileTime == kNumParams || DP::RowsAtCompileTime == Eigen::Dynamic),
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    using T = typename D::Scalar;

    // Unprojection
    const T fu = params[0];
    const T fv = params[1];
    const T u0 = params[2];
    const T v0 = params[3];

    const T k0 = params[4];
    const T k1 = params[5];
    const T k2 = params[6];
    const T k3 = params[7];

    const T un = (uvPixel(0) - u0) / fu;
    const T vn = (uvPixel(1) - v0) / fv;
    const T rth2 = un * un + vn * vn;

    if (rth2 < Sophus::Constants<T>::epsilon() * Sophus::Constants<T>::epsilon()) {
      return Eigen::Matrix<T, 3, 1>(un, vn, T(1.0));
    }

    using std::sqrt;
    const T rth = sqrt(rth2);

    // Use Newtons method to solve for theta.
    T th = CameraNewtonsMethod::initTheta(rth);
    for (int i = 0; i < CameraNewtonsMethod::kMaxIterations; ++i) {
      const T th2 = th * th;
      const T th4 = th2 * th2;
      const T th6 = th4 * th2;
      const T th8 = th4 * th4;

      const T thd = th * (T(1.0) + k0 * th2 + k1 * th4 + k2 * th6 + k3 * th8);

      const T d_thd_wtr_th =
          T(1.0) + T(3.0) * k0 * th2 + T(5.0) * k1 * th4 + T(7.0) * k2 * th6 + T(9.0) * k3 * th8;

      const T step = (thd - rth) / d_thd_wtr_th;
      th -= step;
      if (CameraNewtonsMethod::hasConverged(step)) {
        break;
      }
    }

    using std::tan;
    T radiusUndistorted = tan(th);

    if (radiusUndistorted < T(0.0)) {
      return Eigen::Matrix<T, 3, 1>(
          -radiusUndistorted * un / rth, -radiusUndistorted * vn / rth, T(-1.0));
    }
    return Eigen::Matrix<T, 3, 1>(
        radiusUndistorted * un / rth, radiusUndistorted * vn / rth, T(1.0));
  }

  // also produce unproject_jacobian wrt image pixel
  template <class D, class DP, class DJ1 = Eigen::Matrix<typename D::Scalar, 2, 2>>
  static ARK_HOST_DEVICE Eigen::Matrix<typename D::Scalar, 2, 1> unprojectUnitPlane(
      const Eigen::MatrixBase<D>& uvPixel,
      const Eigen::MatrixBase<DP>& params,
      Eigen::MatrixBase<DJ1>* d_uvPixel = nullptr) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D, 2, 1);

    EIGEN_STATIC_ASSERT(
        DP::ColsAtCompileTime == 1 &&
            (DP::RowsAtCompileTime == kNumParams || DP::RowsAtCompileTime == Eigen::Dynamic),

        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(
        DJ1::ColsAtCompileTime == 2 && DJ1::RowsAtCompileTime == 2,
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    using T = typename D::Scalar;

    const Eigen::Matrix<T, 3, 1> ray = unproject(uvPixel, params);

    if (d_uvPixel) {
      // at this point: uvUnitPlane, project it again to get the jacobian wrt pixel
      Eigen::Matrix<T, 2, 3> J_point;
      project(ray, params, &J_point);
      (*d_uvPixel) = J_point.template topLeftCorner<2, 2>() / params[0];
      // eval() needed for CUDA
      (*d_uvPixel) = (*d_uvPixel).inverse().eval() / params[0];
    }

    const Eigen::Matrix<T, 2, 1> uvUnitPlane = ray.template head<2>();
    return uvUnitPlane;
  }

  // Return scaled parameters to cope with scaled image.
  template <class Scalar, class DP>
  ARK_HOST_DEVICE static void scaleParams(Scalar s, Eigen::DenseBase<DP>& params) {
    EIGEN_STATIC_ASSERT(
        DP::ColsAtCompileTime == 1 &&
            (DP::RowsAtCompileTime == kNumParams || DP::RowsAtCompileTime == Eigen::Dynamic),
        THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    using T = typename DP::Scalar;
    const typename DP::Scalar& scale = s;
    params[0] *= scale;
    params[1] *= scale;
    params[2] = scale * (params[2] + 0.5) - T(0.5);
    params[3] = scale * (params[3] + 0.5) - T(0.5);
    // Distortion cooefficients aren't effected by scale
  }

  // Subtract values from optical center, useful when sensor plane is cropped
  template <typename DP>
  ARK_HOST_DEVICE static void
  subtractFromOrigin(typename DP::Scalar u, typename DP::Scalar v, Eigen::DenseBase<DP>& params) {
    params[2] -= u;
    params[3] -= v;
  }
};

// Basically same as KannalaBrandtK3Projection, except for the points behind the camera are
// projected symmetrically to the positive sphere.
// This model is symmetric such that points represented in inverse depth project to similar
// points when moving from positive depth to negative depth. i.e. the point (1,1,1) and (-1,-1,-1)
// project to the same pixel. That was a requirement for VIO and since we didn't have any camera
// lens with FoV > 180 deg we decided to go for this symmetry. As a result if a point at the border
// of the image sensor (i.e. 0px ,0px) would unproject to a point with z < 0 (which can happen even
// for lenses that are FoV<180 deg, because the polynomial we use to approximate distortion is only
// calibrated in the part of the image where we have measurements and does whatever in the part we
// don't have measurements) it might unproject to a wrong 3d bearing!
// TODO(burrimi) Introduce the symmetry in the wrappers. Projection models should only be reference
// implementations without such assumptions to also allow experiments with FoV > 180.
// https://fb.workplace.com/groups/2092693450825312/permalink/2192894444138545/
class KannalaBrandtK3ProjectionSymmetric {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kNumParams = 8;
  static constexpr char kName[] = "KannalaBrandtK3Symmetric";
  static constexpr int kNumDistortionParams = 4;
  static constexpr int kFocalXIdx = 0;
  static constexpr int kFocalYIdx = 1;
  static constexpr int kPrincipalPointColIdx = 2;
  static constexpr int kPrincipalPointRowIdx = 3;
  static constexpr bool kIsFisheye = true;
  static constexpr bool kHasAnalyticalProjection = true;

  // Takes in 3-point ``pointOptical`` in the local reference frame of the camera and projects it
  // onto the image plan. Since this is a symmetric model, points with negative z are projected to
  // the positive sphere! i.e project(1,1,-1) == project(-1,-1,1)
  //
  // Precondition: pointOptical.z() != 0.
  //
  // Return 2-point in the image plane.
  //
  template <
      class D,
      class DP,
      class DJ1 = Eigen::Matrix<typename D::Scalar, 2, 3>,
      class DJ2 = Eigen::Matrix<typename D::Scalar, 2, kNumParams>>
  static ARK_HOST_DEVICE Eigen::Matrix<typename D::Scalar, 2, 1> project(
      const Eigen::MatrixBase<D>& pointOptical,
      const Eigen::MatrixBase<DP>& params,
      Eigen::MatrixBase<DJ1>* d_point = nullptr,
      Eigen::MatrixBase<DJ2>* d_params = nullptr) {
    using T = typename D::Scalar;

    static_assert(
        D::RowsAtCompileTime == 3 && D::ColsAtCompileTime == 1,
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");
    static_assert(
        DP::ColsAtCompileTime == 1 &&
            (DP::RowsAtCompileTime == kNumParams || DP::RowsAtCompileTime == Eigen::Dynamic),
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");
    static_assert(
        (DJ1::ColsAtCompileTime == 3 || DJ1::ColsAtCompileTime == Eigen::Dynamic) &&
            (DJ1::RowsAtCompileTime == 2 || DJ1::RowsAtCompileTime == Eigen::Dynamic),
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");
    static_assert(
        (DJ2::ColsAtCompileTime == kNumParams || DJ2::ColsAtCompileTime == Eigen::Dynamic) &&
            (DJ2::RowsAtCompileTime == 2 || DJ2::RowsAtCompileTime == Eigen::Dynamic),
        "THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE");

    SOPHUS_ENSURE(pointOptical.z() != T(0), "z(%) must not be zero.", pointOptical.z());

    const T zInv = T(1.0) / pointOptical[2];
    const Eigen::Matrix<T, 2, 1> pointUnitPlane = pointOptical.template head<2>() * zInv;

    // Focal length and principal point
    const Eigen::Matrix<T, 2, 1> ff = params.template head<2>();
    const Eigen::Matrix<T, 2, 1> pp = params.template segment<2>(2);

    const T k0 = params[4];
    const T k1 = params[5];
    const T k2 = params[6];
    const T k3 = params[7];

    const T radiusSquared = pointUnitPlane.squaredNorm();
    using std::atan;
    using std::sqrt;
    const T radius = sqrt(radiusSquared);

    if (radius > Sophus::Constants<T>::epsilon()) {
      const T radiusInverse = T(1.0) / radius;
      const T theta = atan(radius);
      const T theta2 = theta * theta;
      const T theta4 = theta2 * theta2;
      const T theta6 = theta4 * theta2;
      const T theta8 = theta4 * theta4;
      const T rDistorted = theta * (T(1.0) + k0 * theta2 + k1 * theta4 + k2 * theta6 + k3 * theta8);
      const T scaling = rDistorted * radiusInverse;

      if (d_point) {
        const T radiusSquaredInverse = radiusInverse * radiusInverse;
        const T xNormalizedSquared = pointUnitPlane(0) * pointUnitPlane(0) * radiusSquaredInverse;
        const T yNormalizedSquared = pointUnitPlane(1) * pointUnitPlane(1) * radiusSquaredInverse;
        const T xyNormalized = pointUnitPlane(0) * pointUnitPlane(1) * radiusSquaredInverse;
        const T normSquared = T(1.0) + radiusSquared;
        const T rDistortedDerivative = T(1.0) + T(3.0) * k0 * theta2 + T(5.0) * k1 * theta4 +
            T(7.0) * k2 * theta6 + T(9.0) * k3 * theta8;
        const T Jdistortion01 = rDistortedDerivative / normSquared - scaling;
        const T Jdistortion00 = rDistortedDerivative / normSquared - radiusInverse * rDistorted;

        (*d_point)(0, 0) = xNormalizedSquared * Jdistortion00 + scaling;
        (*d_point)(0, 1) = xyNormalized * Jdistortion01;
        (*d_point)(0, 2) =
            -zInv * (pointOptical[0] * (*d_point)(0, 0) + pointOptical[1] * (*d_point)(0, 1));
        (*d_point)(1, 0) = (*d_point)(0, 1);

        (*d_point)(1, 1) = yNormalizedSquared * Jdistortion00 + scaling;
        (*d_point)(1, 2) =
            -zInv * (pointOptical[0] * (*d_point)(1, 0) + pointOptical[1] * (*d_point)(1, 1));

        (*d_point) = ff.asDiagonal() * zInv * (*d_point);
      }
      using std::pow;
      if (d_params) {
        const T xScaled = pointUnitPlane[0] * params[0] * radiusInverse;
        const T yScaled = pointUnitPlane[1] * params[1] * radiusInverse;

        const T theta3 = theta * theta2;
        const T theta5 = theta3 * theta2;
        const T theta7 = theta5 * theta2;
        const T theta9 = theta7 * theta2;

        (*d_params)(0, 0) = pointUnitPlane[0] * scaling;
        (*d_params)(0, 1) = T(0.0);
        (*d_params)(0, 2) = T(1.0);
        (*d_params)(0, 3) = T(0.0);
        (*d_params)(0, 4) = xScaled * theta3;
        (*d_params)(0, 5) = xScaled * theta5;
        (*d_params)(0, 6) = xScaled * theta7;
        (*d_params)(0, 7) = xScaled * theta9;
        (*d_params)(1, 0) = T(0.0);
        (*d_params)(1, 1) = pointUnitPlane[1] * scaling;
        (*d_params)(1, 2) = T(0.0);
        (*d_params)(1, 3) = T(1.0);
        (*d_params)(1, 4) = yScaled * theta3;
        (*d_params)(1, 5) = yScaled * theta5;
        (*d_params)(1, 6) = yScaled * theta7;
        (*d_params)(1, 7) = yScaled * theta9;
      }

      const Eigen::Matrix<T, 2, 1> px = scaling * ff.cwiseProduct(pointUnitPlane) + pp;

      return px;
    } else {
      // linearize r around radius=0
      if (d_point) {
        // clang-format off
          (*d_point) << ff.x() * zInv, T(0.0), -ff.x() * pointUnitPlane(0) * zInv,
                      T(0.0), ff.y() * zInv, -ff.y() * pointUnitPlane(1) * zInv;
        // clang-format on
      }
      if (d_params) {
        (*d_params)(0, 0) = pointUnitPlane(0);
        (*d_params)(0, 1) = T(0.0);
        (*d_params)(0, 2) = T(1.0);
        (*d_params)(0, 3) = T(0.0);

        (*d_params)(1, 0) = T(0.0);
        (*d_params)(1, 1) = pointUnitPlane(1);
        (*d_params)(1, 2) = T(0.0);
        (*d_params)(1, 3) = T(1.0);
        (*d_params).template rightCols<4>().setZero();
      }
      const Eigen::Matrix<T, 2, 1> px = ff.cwiseProduct(pointUnitPlane) + pp;

      return px;
    }
  }

  // Same as the unproject in KannalaBrandtK3Projection
  template <typename D, typename DP>
  static ARK_HOST_DEVICE Eigen::Matrix<typename D::Scalar, 3, 1> unproject(
      const Eigen::MatrixBase<D>& uvPixel,
      const Eigen::MatrixBase<DP>& params) {
    return KannalaBrandtK3Projection::unproject(uvPixel, params);
  }

  // same as unprojectUnitPlane in KannalaBrandtK3Projection
  template <class D, class DP, class DJ1 = Eigen::Matrix<typename D::Scalar, 2, 2>>
  static ARK_HOST_DEVICE Eigen::Matrix<typename D::Scalar, 2, 1> unprojectUnitPlane(
      const Eigen::MatrixBase<D>& uvPixel,
      const Eigen::MatrixBase<DP>& params,
      Eigen::MatrixBase<DJ1>* d_uvPixel = nullptr) {
    return KannalaBrandtK3Projection::unprojectUnitPlane(uvPixel, params, d_uvPixel);
  }

  // Return scaled parameters to cope with scaled image.
  template <class Scalar, class DP>
  ARK_HOST_DEVICE static void scaleParams(Scalar /*s*/, Eigen::DenseBase<DP>& params) {
    return KannalaBrandtK3Projection::scaleParams(params);
  }
};

} // namespace sensors
} // namespace datatools
} // namespace ark
