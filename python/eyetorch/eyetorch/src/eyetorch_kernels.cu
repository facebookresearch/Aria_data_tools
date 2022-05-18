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

#include <Eigen/Dense>

#include <c10/cuda/CUDAGuard.h>

#include <eyetorch_kernels.h>
#include <models/dispatch.h>
#include <typedefs.h>

namespace eyetorch {

static constexpr int BLOCK_SIZE = 256;

inline int getGridSize(const int N) {
  return (N + BLOCK_SIZE - 1) / BLOCK_SIZE;
}

template <typename ScalarT, typename ProjectorT>
__global__ void project_forward_kernel(
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* points,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    typename TypeDefs<ScalarT, ProjectorT>::Point2D* projections,
    const int N) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x) {
    projections[i] = ProjectorT::project(points[i], calibParams[i]);
  }
}

template <typename ScalarT, typename ProjectorT>
void project_forward_gpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* points,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    typename TypeDefs<ScalarT, ProjectorT>::Point2D* projections,
    const int N) {
  project_forward_kernel<ScalarT, ProjectorT>
      <<<getGridSize(N), BLOCK_SIZE>>>(points, calibParams, projections, N);
}

template <typename ScalarT, typename ProjectorT>
__global__ void project_backward_kernel(
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* points,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* gradProjections,
    typename TypeDefs<ScalarT, ProjectorT>::Point3D* gradPoints,
    typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* gradParams,
    const int N) {
  using T = TypeDefs<ScalarT, ProjectorT>;

  using JPoint2DxPoint3D = typename T::JPoint2DxPoint3D;
  using JPoint2DxCalibParamVec = typename T::JPoint2DxCalibParamVec;

  JPoint2DxPoint3D jProjectionByPoint;
  JPoint2DxCalibParamVec jProjectionByParams;

  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x) {
    ProjectorT::computeProjectJacobians(
        points[i], calibParams[i], jProjectionByPoint, jProjectionByParams);

    gradPoints[i] = jProjectionByPoint.transpose() * gradProjections[i];
    gradParams[i] = jProjectionByParams.transpose() * gradProjections[i];
  }
}

template <typename ScalarT, typename ProjectorT>
void project_backward_gpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* points,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* gradProjections,
    typename TypeDefs<ScalarT, ProjectorT>::Point3D* gradPoints,
    typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* gradParams,
    const int N) {
  project_backward_kernel<ScalarT, ProjectorT><<<getGridSize(N), BLOCK_SIZE>>>(
      points, calibParams, gradProjections, gradPoints, gradParams, N);
}

template <typename ScalarT, typename ProjectorT>
__global__ void unproject_forward_kernel(
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* pixels,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    typename TypeDefs<ScalarT, ProjectorT>::Point3D* rays,
    const int N) {
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x) {
    rays[i] = ProjectorT::unproject(pixels[i], calibParams[i]);
  }
}

template <typename ScalarT, typename ProjectorT>
void unproject_forward_gpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* pixels,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    typename TypeDefs<ScalarT, ProjectorT>::Point3D* rays,
    const int N) {
  unproject_forward_kernel<ScalarT, ProjectorT>
      <<<getGridSize(N), BLOCK_SIZE>>>(pixels, calibParams, rays, N);
}

template <typename ScalarT, typename ProjectorT>
__global__ void unproject_backward_kernel(
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* pixels,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* gradRays,
    typename TypeDefs<ScalarT, ProjectorT>::Point2D* gradPixels,
    typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* gradParams,
    const int N) {
  using T = TypeDefs<ScalarT, ProjectorT>;

  using JPoint2DxPoint2D = typename T::JPoint2DxPoint2D;
  using JPoint2DxCalibParamVec = typename T::JPoint2DxCalibParamVec;

  // The projected ray is in 3D. Therefore, the Jacobians for the
  // pixel location and param vector should be 3x2 and 3xK,
  // respectively. However, because EyeTorch always unprojects
  // onto the unit plane, the partials w.r.t. the z-coordinate
  // are always zero. It is thus sufficient to compute the
  // 2x2 and 2xK top rows of the Jacobians, with the last row
  // being implicitly zero.
  JPoint2DxPoint2D jRaysByPixels;
  JPoint2DxCalibParamVec jRaysByParams;

  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < N; i += blockDim.x * gridDim.x) {
    ProjectorT::computeUnprojectJacobians(pixels[i], calibParams[i], jRaysByPixels, jRaysByParams);

    gradPixels[i] = jRaysByPixels.transpose() * gradRays[i].template head<2>();
    gradParams[i] = jRaysByParams.transpose() * gradRays[i].template head<2>();
  }
}

template <typename ScalarT, typename ProjectorT>
void unproject_backward_gpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* pixels,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* gradRays,
    typename TypeDefs<ScalarT, ProjectorT>::Point2D* gradPixels,
    typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* gradParams,
    const int N) {
  unproject_backward_kernel<ScalarT, ProjectorT>
      <<<getGridSize(N), BLOCK_SIZE>>>(pixels, calibParams, gradRays, gradPixels, gradParams, N);
}

#define INSTANTIATE_GPU_FUNCTIONS(ProjectionModel)               \
  template void project_forward_gpu<float, ProjectionModel>(     \
      const typename TypeDefsF<ProjectionModel>::Point3D*,       \
      const typename TypeDefsF<ProjectionModel>::CalibParamVec*, \
      typename TypeDefsF<ProjectionModel>::Point2D*,             \
      const int);                                                \
  template void project_forward_gpu<double, ProjectionModel>(    \
      const typename TypeDefsD<ProjectionModel>::Point3D*,       \
      const typename TypeDefsD<ProjectionModel>::CalibParamVec*, \
      typename TypeDefsD<ProjectionModel>::Point2D*,             \
      const int);                                                \
  template void project_backward_gpu<float, ProjectionModel>(    \
      const typename TypeDefsF<ProjectionModel>::Point3D*,       \
      const typename TypeDefsF<ProjectionModel>::CalibParamVec*, \
      const typename TypeDefsF<ProjectionModel>::Point2D*,       \
      typename TypeDefsF<ProjectionModel>::Point3D*,             \
      typename TypeDefsF<ProjectionModel>::CalibParamVec*,       \
      const int);                                                \
  template void project_backward_gpu<double, ProjectionModel>(   \
      const typename TypeDefsD<ProjectionModel>::Point3D*,       \
      const typename TypeDefsD<ProjectionModel>::CalibParamVec*, \
      const typename TypeDefsD<ProjectionModel>::Point2D*,       \
      typename TypeDefsD<ProjectionModel>::Point3D*,             \
      typename TypeDefsD<ProjectionModel>::CalibParamVec*,       \
      const int);                                                \
  template void unproject_forward_gpu<float, ProjectionModel>(   \
      const typename TypeDefsF<ProjectionModel>::Point2D*,       \
      const typename TypeDefsF<ProjectionModel>::CalibParamVec*, \
      typename TypeDefsF<ProjectionModel>::Point3D*,             \
      const int);                                                \
  template void unproject_forward_gpu<double, ProjectionModel>(  \
      const typename TypeDefsD<ProjectionModel>::Point2D*,       \
      const typename TypeDefsD<ProjectionModel>::CalibParamVec*, \
      typename TypeDefsD<ProjectionModel>::Point3D*,             \
      const int);                                                \
  template void unproject_backward_gpu<float, ProjectionModel>(  \
      const typename TypeDefsF<ProjectionModel>::Point2D*,       \
      const typename TypeDefsF<ProjectionModel>::CalibParamVec*, \
      const typename TypeDefsF<ProjectionModel>::Point3D*,       \
      typename TypeDefsF<ProjectionModel>::Point2D*,             \
      typename TypeDefsF<ProjectionModel>::CalibParamVec*,       \
      const int);                                                \
  template void unproject_backward_gpu<double, ProjectionModel>( \
      const typename TypeDefsD<ProjectionModel>::Point2D*,       \
      const typename TypeDefsD<ProjectionModel>::CalibParamVec*, \
      const typename TypeDefsD<ProjectionModel>::Point3D*,       \
      typename TypeDefsD<ProjectionModel>::Point2D*,             \
      typename TypeDefsD<ProjectionModel>::CalibParamVec*,       \
      const int)

CALL_PREPROC_MACRO_FOR_EACH_PROJECTOR(INSTANTIATE_GPU_FUNCTIONS);

} // namespace eyetorch
