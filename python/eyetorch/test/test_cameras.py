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

import math
import unittest
from typing import List

import eyetorch
import torch
from parameterized import parameterized_class


def get_rtol_atol(dtype):
    if dtype == torch.float32:
        return 1e-3, 1e-5
    elif dtype == torch.float64:
        return 1e-5, 1e-8


def all_combinations(l1, l2, l3):
    return [(i1, i2, i3) for i1 in l1 for i2 in l2 for i3 in l3]


all_models = [eyetorch.Pinhole, eyetorch.KannalaBrandt3, eyetorch.Fisheye624]
all_dtypes = [torch.float32, torch.float64]
all_devices = [torch.device("cpu")] + (
    [torch.device("cuda")] if torch.cuda.is_available() else []
)


@parameterized_class(
    ("model", "dtype", "device"), all_combinations(all_models, all_dtypes, all_devices)
)
class TestCamera(unittest.TestCase):
    def __test_cycle_template(
        self, image_size: List[int], params_size: List[int], points_size: List[int]
    ):
        model = self.model
        device = self.device
        dtype = self.dtype

        with self.subTest(
            model=model,
            device=device,
            dtype=dtype,
            params_size=params_size,
            points_size=points_size,
        ):

            # small distortion parameters
            camera_params = 0.05 * torch.randn(
                *params_size, model.num_params, device=device, dtype=dtype
            )
            # typical focal and principal point
            diag = math.sqrt(image_size[0] ** 2 + image_size[1] ** 2)
            camera_params[..., model.focal_x_idx] += 0.7 * diag
            camera_params[..., model.focal_y_idx] += 0.7 * diag
            camera_params[..., model.principal_point_x_idx] += image_size[0] / 2.0
            camera_params[..., model.principal_point_y_idx] += image_size[1] / 2.0

            camera = model(camera_params)

            # randomly select pixels in the image
            pixels = torch.stack(
                [
                    size * torch.rand(*points_size, dtype=dtype, device=device)
                    for size in image_size
                ],
                dim=-1,
            )

            # project to the unit plane
            points = camera.unproject(pixels)

            # randomly assign depths
            points = points * (
                1.0 + 4.0 * torch.rand(*points_size, 1, dtype=dtype, device=device)
            )

            # reproject the points
            pixels2 = camera.project(points)

            # make sure the reprojected points match the original points
            self.assertTrue(torch.allclose(pixels, pixels2, *get_rtol_atol(dtype)))

    def test_cycle(self):

        self.__test_cycle_template([720, 720], [], [10])
        self.__test_cycle_template([320, 240], [3], [1])
        self.__test_cycle_template([1920, 1080], [5, 1], [1, 8])
        self.__test_cycle_template([640, 480], [4, 1], [4, 10])
        self.__test_cycle_template([640, 480], [4, 10], [4, 10])
        self.__test_cycle_template([800, 600], [1, 3], [10, 1])
        self.__test_cycle_template([1280, 960], [3, 1, 2], [3, 4, 1])
        self.__test_cycle_template([800, 800], [1, 1, 1, 1, 2], [2, 2, 2, 2, 1])

    def __test_grad_template(
        self, image_size: List[int], params_size: List[int], points_size: List[int]
    ):
        model = self.model
        device = self.device
        dtype = self.dtype

        with self.subTest(
            model=model,
            device=device,
            dtype=dtype,
            params_size=params_size,
            points_size=points_size,
        ):

            # small distortion parameters
            camera_params = 0.05 * torch.randn(
                *params_size, model.num_params, device=device, dtype=dtype
            )
            # typical focal and principal point
            diag = math.sqrt(image_size[0] ** 2 + image_size[1] ** 2)
            camera_params[..., model.focal_x_idx] += 0.7 * diag
            camera_params[..., model.focal_y_idx] += 0.7 * diag
            camera_params[..., model.principal_point_x_idx] += image_size[0] / 2.0
            camera_params[..., model.principal_point_y_idx] += image_size[1] / 2.0

            camera = model(camera_params)

            # randomly select pixels in the image
            pixels = torch.stack(
                [
                    size * torch.rand(*points_size, dtype=dtype, device=device)
                    for size in image_size
                ],
                dim=-1,
            )
            pixels.requires_grad = True
            if model.unproject_differentiable_by_params:
                camera.requires_grad = True

            def do_unproject(camera, pixels):
                return camera.unproject(pixels)

            self.assertTrue(
                torch.autograd.gradcheck(
                    do_unproject,
                    (camera, pixels),
                )
            )

            camera = camera.clone().detach()
            camera.requires_grad = False
            pixels.requires_grad = False

            points = camera.unproject(pixels)

            points.requires_grad = True
            if model.project_differentiable_by_params:
                camera.requires_grad = True

            def do_project(camera, points):
                return camera.project(points)

            self.assertTrue(
                torch.autograd.gradcheck(
                    do_project,
                    (camera, points),
                )
            )

    def test_grad(self):
        if self.dtype == torch.float32:
            # 32-bit floating point is not precise enough for autograd checks
            return

        self.__test_grad_template([720, 720], [], [10])
        self.__test_grad_template([320, 240], [3], [1])
        self.__test_grad_template([1920, 1080], [5, 1], [1, 8])
        self.__test_grad_template([640, 480], [4, 1], [4, 10])
        self.__test_grad_template([640, 480], [4, 10], [4, 10])
        self.__test_grad_template([800, 600], [1, 3], [10, 1])
        self.__test_grad_template([1280, 960], [3, 1, 2], [3, 4, 1])
        self.__test_grad_template([800, 800], [1, 1, 1, 1, 2], [2, 2, 2, 2, 1])
