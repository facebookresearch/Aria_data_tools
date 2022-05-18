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

import eyetorch_extension
import torch

from .broadcast import check_size_and_broadcast
from .camera_ops import get_projector, get_unprojector


class Camera(torch.Tensor):
    def project(self, points: torch.Tensor) -> torch.Tensor:
        points, params = check_size_and_broadcast(points, self, 3, self.num_params)
        return self.Projector.apply(
            points.flatten(0, -2).contiguous(), params.flatten(0, -2).contiguous()
        ).view(*points.shape[:-1], -1)

    def unproject(self, pixels: torch.Tensor) -> torch.Tensor:
        pixels, params = check_size_and_broadcast(pixels, self, 2, self.num_params)
        return self.Unprojector.apply(
            pixels.flatten(0, -2).contiguous(), params.flatten(0, -2).contiguous()
        ).view(*pixels.shape[:-1], -1)


def camera_class(projection_type_name):
    def camera_class_decorator(cls):
        # fetch the bound C class object from the Python bindings
        c_cls_obj = getattr(eyetorch_extension, projection_type_name)

        # import the bound class objects
        for name, value in vars(c_cls_obj).items():
            if not name.startswith("__"):
                setattr(cls, name, value)

        cls.num_params = c_cls_obj.num_params

        # build autograd functions for this class
        cls.Projector = get_projector(projection_type_name)
        cls.Unprojector = get_unprojector(projection_type_name)

        return cls

    return camera_class_decorator


@camera_class("Pinhole")
class Pinhole(Camera):
    pass


@camera_class("KB3")
class KannalaBrandt3(Camera):
    pass


@camera_class("F624")
class Fisheye624(Camera):
    pass
