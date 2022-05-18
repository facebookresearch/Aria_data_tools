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

#include <torch/torch.h>
#include <typedefs.h>

namespace eyetorch {

template <typename ScalarT, typename ProjectorT>
void project_forward_cpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* points,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    typename TypeDefs<ScalarT, ProjectorT>::Point2D* projections,
    const int N);

template <typename ScalarT, typename ProjectorT>
void project_backward_cpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* points,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* gradProjections,
    typename TypeDefs<ScalarT, ProjectorT>::Point3D* gradPoints,
    typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* gradParams,
    const int N);

template <typename ScalarT, typename ProjectorT>
void unproject_forward_cpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* pixels,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    typename TypeDefs<ScalarT, ProjectorT>::Point3D* rays,
    const int N);

template <typename ScalarT, typename ProjectorT>
void unproject_backward_cpu(
    const typename TypeDefs<ScalarT, ProjectorT>::Point2D* pixels,
    const typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* calibParams,
    const typename TypeDefs<ScalarT, ProjectorT>::Point3D* gradRays,
    typename TypeDefs<ScalarT, ProjectorT>::Point2D* gradPixels,
    typename TypeDefs<ScalarT, ProjectorT>::CalibParamVec* gradParams,
    const int N);

} // namespace eyetorch
