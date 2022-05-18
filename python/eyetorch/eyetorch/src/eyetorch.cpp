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

#include <c10/cuda/CUDAGuard.h>
#include <eyetorch_host.h>
#include <eyetorch_kernels.h>
#include <models/dispatch.h>
#include <torch/extension.h>
#include <typedefs.h>

namespace eyetorch {

template <typename ProjectorT>
torch::Tensor projectForward(torch::Tensor pointsTensor, torch::Tensor calibParamsTensor) {
  // It is assumed all data has been broadcast and flattened, such that
  // points has shape (N, 3).
  const int N = pointsTensor.size(0);
  torch::Tensor projectionsTensor = torch::empty(
      {N, 2},
      torch::TensorOptions()
          .dtype(pointsTensor.dtype())
          .device(pointsTensor.device())
          .layout(torch::kStrided));

  AT_DISPATCH_FLOATING_TYPES(pointsTensor.scalar_type(), "project_forward", [&] {
    using T = TypeDefs<scalar_t, ProjectorT>;

    using Point3D = typename T::Point3D;
    using CalibParamVec = typename T::CalibParamVec;
    using Point2D = typename T::Point2D;

    const Point3D* points = reinterpret_cast<const Point3D*>(pointsTensor.data_ptr<scalar_t>());
    const CalibParamVec* calibParams =
        reinterpret_cast<const CalibParamVec*>(calibParamsTensor.data_ptr<scalar_t>());
    Point2D* projections = reinterpret_cast<Point2D*>(projectionsTensor.data_ptr<scalar_t>());

    if (pointsTensor.device().type() == torch::DeviceType::CPU) {
      project_forward_cpu<scalar_t, ProjectorT>(points, calibParams, projections, N);
    } else {
      const at::cuda::OptionalCUDAGuard deviceGuard(device_of(pointsTensor));
      project_forward_gpu<scalar_t, ProjectorT>(points, calibParams, projections, N);
    }
  });

  return projectionsTensor;
}

template <typename ProjectorT>
std::tuple<torch::Tensor, torch::Tensor> projectBackward(
    torch::Tensor pointsTensor,
    torch::Tensor calibParamsTensor,
    torch::Tensor gradProjectionsTensor) {
  const int N = pointsTensor.size(0);
  torch::Tensor gradPointsTensor = torch::empty_like(pointsTensor);
  torch::Tensor gradParamsTensor = torch::empty_like(calibParamsTensor);

  AT_DISPATCH_FLOATING_TYPES(pointsTensor.scalar_type(), "project_backward", [&] {
    using T = TypeDefs<scalar_t, ProjectorT>;

    using Point3D = typename T::Point3D;
    using CalibParamVec = typename T::CalibParamVec;
    using Point2D = typename T::Point2D;

    const Point3D* points = reinterpret_cast<const Point3D*>(pointsTensor.data_ptr<scalar_t>());
    const CalibParamVec* calibParams =
        reinterpret_cast<const CalibParamVec*>(calibParamsTensor.data_ptr<scalar_t>());
    const Point2D* gradProjections =
        reinterpret_cast<const Point2D*>(gradProjectionsTensor.data_ptr<scalar_t>());
    Point3D* gradPoints = reinterpret_cast<Point3D*>(gradPointsTensor.data_ptr<scalar_t>());
    CalibParamVec* gradParams =
        reinterpret_cast<CalibParamVec*>(gradParamsTensor.data_ptr<scalar_t>());

    if (pointsTensor.device().type() == torch::DeviceType::CPU) {
      project_backward_cpu<scalar_t, ProjectorT>(
          points, calibParams, gradProjections, gradPoints, gradParams, N);
    } else {
      const at::cuda::OptionalCUDAGuard deviceGuard(device_of(pointsTensor));
      project_backward_gpu<scalar_t, ProjectorT>(
          points, calibParams, gradProjections, gradPoints, gradParams, N);
    }
  });

  return {gradPointsTensor, gradParamsTensor};
}

template <typename ProjectorT>
torch::Tensor unprojectForward(torch::Tensor pixelsTensor, torch::Tensor calibParamsTensor) {
  // It is assumed all data has been broadcast and flattened, such that
  // pixels has shape (N, 3).
  const int N = pixelsTensor.size(0);
  torch::Tensor raysTensor = torch::empty(
      {N, 3},
      torch::TensorOptions()
          .dtype(pixelsTensor.dtype())
          .device(pixelsTensor.device())
          .layout(torch::kStrided));

  AT_DISPATCH_FLOATING_TYPES(pixelsTensor.scalar_type(), "unproject_forward", [&] {
    using T = TypeDefs<scalar_t, ProjectorT>;

    using Point3D = typename T::Point3D;
    using CalibParamVec = typename T::CalibParamVec;
    using Point2D = typename T::Point2D;

    const Point2D* pixels = reinterpret_cast<const Point2D*>(pixelsTensor.data_ptr<scalar_t>());
    const CalibParamVec* calibParams =
        reinterpret_cast<const CalibParamVec*>(calibParamsTensor.data_ptr<scalar_t>());
    Point3D* rays = reinterpret_cast<Point3D*>(raysTensor.data_ptr<scalar_t>());

    if (pixelsTensor.device().type() == torch::DeviceType::CPU) {
      unproject_forward_cpu<scalar_t, ProjectorT>(pixels, calibParams, rays, N);
    } else {
      const at::cuda::OptionalCUDAGuard deviceGuard(device_of(pixelsTensor));
      unproject_forward_gpu<scalar_t, ProjectorT>(pixels, calibParams, rays, N);
    }
  });

  return raysTensor;
}

template <typename ProjectorT>
std::tuple<torch::Tensor, torch::Tensor> unprojectBackward(
    torch::Tensor pixelsTensor,
    torch::Tensor calibParamsTensor,
    torch::Tensor gradRaysTensor) {
  const int N = pixelsTensor.size(0);
  torch::Tensor gradPixelsTensor = torch::empty_like(pixelsTensor);
  torch::Tensor gradParamsTensor = torch::empty_like(calibParamsTensor);

  AT_DISPATCH_FLOATING_TYPES(pixelsTensor.scalar_type(), "unproject_backward", [&] {
    using T = TypeDefs<scalar_t, ProjectorT>;

    using Point3D = typename T::Point3D;
    using CalibParamVec = typename T::CalibParamVec;
    using Point2D = typename T::Point2D;

    const Point2D* pixels = reinterpret_cast<const Point2D*>(pixelsTensor.data_ptr<scalar_t>());
    const CalibParamVec* calibParams =
        reinterpret_cast<const CalibParamVec*>(calibParamsTensor.data_ptr<scalar_t>());
    const Point3D* gradRays = reinterpret_cast<const Point3D*>(gradRaysTensor.data_ptr<scalar_t>());
    Point2D* gradPixels = reinterpret_cast<Point2D*>(gradPixelsTensor.data_ptr<scalar_t>());
    CalibParamVec* gradParams =
        reinterpret_cast<CalibParamVec*>(gradParamsTensor.data_ptr<scalar_t>());

    if (pixelsTensor.device().type() == torch::DeviceType::CPU) {
      unproject_backward_cpu<scalar_t, ProjectorT>(
          pixels, calibParams, gradRays, gradPixels, gradParams, N);
    } else {
      const at::cuda::OptionalCUDAGuard deviceGuard(device_of(pixelsTensor));
      unproject_backward_gpu<scalar_t, ProjectorT>(
          pixels, calibParams, gradRays, gradPixels, gradParams, N);
    }
  });

  return {gradPixelsTensor, gradParamsTensor};
}

#define BIND_FUNCTIONS(ProjectionModel, ModelName)                                              \
  m.def("project_forward_" #ModelName, &projectForward<ProjectionModel>);                       \
  m.def("project_backward_" #ModelName, &projectBackward<ProjectionModel>);                     \
  m.def("unproject_forward_" #ModelName, &unprojectForward<ProjectionModel>);                   \
  m.def("unproject_backward_" #ModelName, &unprojectBackward<ProjectionModel>);                 \
  pybind11::class_<ProjectionModel>(m, #ModelName)                                              \
      .def_readonly_static("num_params", &ProjectionModel::kNumParams)                          \
      .def_readonly_static("focal_x_idx", &ProjectionModel::kFocalXIdx)                         \
      .def_readonly_static("focal_y_idx", &ProjectionModel::kFocalYIdx)                         \
      .def_readonly_static("principal_point_x_idx", &ProjectionModel::kPrincipalPointXIdx)      \
      .def_readonly_static("principal_point_y_idx", &ProjectionModel::kPrincipalPointYIdx)      \
      .def_readonly_static(                                                                     \
          "project_differentiable_by_params", &ProjectionModel::kProjectDifferentiableByParams) \
      .def_readonly_static(                                                                     \
          "unproject_differentiable_by_params",                                                 \
          &ProjectionModel::kUnprojectDifferentiableByParams)

PYBIND11_MODULE(eyetorch_extension, m) {
  CALL_PREPROC_MACRO_FOR_EACH_PROJECTOR_WITH_NAME(BIND_FUNCTIONS);
}

} // namespace eyetorch
