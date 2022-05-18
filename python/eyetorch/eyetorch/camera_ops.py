# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Tuple

import eyetorch_extension
import torch


class Projector(torch.autograd.Function):
    @classmethod
    def forward(
        cls, ctx, points: torch.Tensor, camera_params: torch.Tensor
    ) -> torch.Tensor:
        ctx.save_for_backward(points, camera_params)
        return cls.project_forward(points, camera_params)

    @classmethod
    def backward(
        cls, ctx, grad_output: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        points, camera_params = ctx.saved_tensors
        return cls.project_backward(points, camera_params, grad_output.contiguous())


class Unprojector(torch.autograd.Function):
    @classmethod
    def forward(
        cls, ctx, pixels: torch.Tensor, camera_params: torch.Tensor
    ) -> torch.Tensor:
        ctx.save_for_backward(pixels, camera_params)
        return cls.unproject_forward(pixels, camera_params)

    @classmethod
    def backward(
        cls, ctx, grad_output: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        pixels, camera_params = ctx.saved_tensors
        return cls.unproject_backward(pixels, camera_params, grad_output.contiguous())


def get_projector(projection_type_name):
    class _Projector(Projector):
        project_forward = getattr(
            eyetorch_extension, f"project_forward_{projection_type_name}"
        )
        project_backward = getattr(
            eyetorch_extension, f"project_backward_{projection_type_name}"
        )

    return _Projector


def get_unprojector(projection_type_name):
    class _Unprojector(Unprojector):
        unproject_forward = getattr(
            eyetorch_extension, f"unproject_forward_{projection_type_name}"
        )
        unproject_backward = getattr(
            eyetorch_extension, f"unproject_backward_{projection_type_name}"
        )

    return _Unprojector
