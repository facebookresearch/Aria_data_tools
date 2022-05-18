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

from typing import List, Tuple

import torch


def expand_to_match_ndim(shorter: torch.Tensor, longer: torch.Tensor) -> torch.Tensor:
    """
    Given two tensors, one shorter and one longer, this function returns a view
    of the shorter tensor with the same number of dimensions as the longer. It is
    created by prepending single-element dimensions to the Tensor. The function
    does not check that the tensor passed as `shorter` does in fact have fewer
    dimensions than the tensor passed in as `longer`.
    """
    return shorter.view(*([1] * (longer.ndim - shorter.ndim)), *shorter.shape)


def check_size_and_broadcast(
    a: torch.Tensor, b: torch.Tensor, intrinsic_size_a: int, intrinsic_size_b: int
) -> Tuple[torch.Tensor, torch.Tensor]:
    """
    Performs specialized broadcasting of two Tensors where each Tensor has an
    "intrinsic size". For example, a pixel in an image has an intrinsic size of
    2 and a set of calibration parameters for a Pinhole projection model has an
    intrinsic size of 4. This function verifies that the last dimension of each
    Tensor argument matches the expected intrinsic size, and then broadcasts
    across the remainder of the dimensions.

    For example, if a is a Tensor of pixel indices of shape [1, 16, 2] and b is
    a Tensor of Pinhole projection parameters of shape [8, 1, 4], this function
    will return a new pixel Tensor with shape [8, 16, 2] and a new parameter
    Tensor with shape [8, 16, 4]. These can then be passed to the unproject
    operator of the Pinhole projector, for example.

    Note that broadcasting is performed with the 'expand' function, such that
    multiple indices in the returned Tensors may reference the same memory.
    See the documentation for 'torch.Tensor.expand' for more information, and
    be sure to call 'contiguous' on the returned Tensors if this is not a
    desireable property for the use case.
    """

    if len(a.shape) == 0 or a.shape[-1] != intrinsic_size_a:
        raise RuntimeError("Incorrect extrinsic dimensionality in first slot.")
    if len(b.shape) == 0 or b.shape[-1] != intrinsic_size_b:
        raise RuntimeError("Incorrect extrinsic dimensionality in second slot.")

    if a.ndim > b.ndim:
        num_nonintrinsic_axes: int = a.ndim - 1
        broadcast_shape: List[int] = list(a.shape[:-1])
        b = expand_to_match_ndim(b, a)
    else:
        num_nonintrinsic_axes: int = b.ndim - 1
        broadcast_shape: List[int] = list(b.shape[:-1])
        a = expand_to_match_ndim(a, b)

    for i in range(num_nonintrinsic_axes):
        if a.shape[i] == b.shape[i] or a.shape[i] == 1 or b.shape[i] == 1:
            broadcast_shape[i] = max(a.shape[i], b.shape[i])
        else:
            raise RuntimeError("Tensors cannot be broadcast.")

    a = a.expand(*broadcast_shape, intrinsic_size_a)
    b = b.expand(*broadcast_shape, intrinsic_size_b)

    return a, b
