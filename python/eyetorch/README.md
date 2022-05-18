# EyeTorch
## Batched Projection Models for PyTorch

EyeTorch is a PyTorch extension that enables easy projection and un-projection of batches of points using a variety of linear and non-linear projection models.

## Projection Models
EyeTorch leverages  currently supports three camera projection models, each implemented as a Python class:
- `Pinhole`: The standard linear pinhole projection model.
- `KannalaBrandt3`:
- `Fisheye624`:

In order to maximize simplicity, flexibility, and portability, EyeTorch projection models **are** `torch.Tensor` objects. They can be used anywhere and in any way that a `torch.Tensor` object can be used, and are created in the same way `torch.Tensor` objects are created. PyTorch operators applied to an EyeTorch model will produce an EyeTorch model of the same type.

Each projection model provides two operators in addition to those it inherits from `torch.Tensor`. The projection operator maps points in 3D to points on a 2D image plane. The un-projection operator maps points on a 2D image plane to rays in 3D.

While EyeTorch projection model objects are simply `torch.Tensor` objects, they must maintain one invariant to be used for projection and un-projection of points: they must maintain a particular 'intrinsic size' in their last dimension. The intrinsic size is the number of parameters in the projection model. This is provided as a class attribute with the name `num_params`. This invariant is checked before applying the projection or un-projection operator but it is otherwise up to the user to maintain it.

## Simple Example Usage
The following shows a simple example where 100 random 3D points are projected using a single Pinhole camera model.
```python
import eyetorch

camera = eyetorch.Pinhole([475.0, 475.0, 320.0, 240.0])
points_3d = torch.randn(100, 3) = torch.tensor([0.0, 0.0, 4.0])

points_2d = camera.project(points_3d)
```

## Tensor Size Requirements
As discussed above, it is assumed that each EyeTorch object maintains its intrinsic size in the last dimension. Points tensor also have an intrinsic size that is expected and checked by the projection and un-projection operators: points passed to `project` are expected to have intrinsic size 3 in the last dimension and points passed to `unproject` are expected to have intrinsic size 2. Outputs of `project` have intrinsic size 2 and outputs of `unproject` have intrinsic size 3.

EyeTorch supports full broadcasting of both the camera parameters and the points tensors. For example, if a projection model has size `M x 1 x K` (where `K` is the model intrinsic size) and a points tensor has size `N x 3`, projecting the points will result in a tensor with size `M x N x 2` representing the projection of each point with each projection model.

## Differentiability
Eyetorch aims to provide differentiable operators wherever possible. However, it is not always possible to compute analytic partial derivatives for more complex non-linear projection models. Thus EyeTorch provides partial support for automatic differentiation in PyTorch. The provided projection models all implement differentiation with respect to the input points for both projection and un-projection operators, and all implement differentiation with respect to the parameters for the projection operator. However, only the Pinhole model supports differentiation of un-projection with respect to the calibration parameters. This is represented in graphical form below, using the notation 'P(x; K)' to denote the projection of points x with parameters K and 'U(x; K)' to denote the un-projection of points x with parameters K.

|  Model  |  &#x2202;P / &#x2202;x | &#x2202;P / &#x2202;K | &#x2202;U / &#x2202;x | &#x2202;U / &#x2202;K |
| ------- | ------- | ------- | ------- | ------- |
| `Pinhole` |  :heavy_check_mark: |   :heavy_check_mark:   |   :heavy_check_mark:   |   :heavy_check_mark:   |
| `KannalBrandt3` | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :x: |
| `Fisheye624` | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :x: |

For convenience, each projection model class defines boolean attributes named `project_differentiable_by_params` and `unproject_differentiable_by_params` that enable one to dynamically query whether a particular model supports a particular type of differentiability.


## Conventions

TODO

## Installation

TODO
