---
sidebar_position: 8
id: multiple-recordings
title: Access Time Synchronized Data From Multiple Project Aria Recordings
---

#  Access Time Synchronized Data From Multiple Project Aria Recordings

To access time synchronized data from multiple Project Aria recordings, we created the EyeTorch extension in Pytorch. EyeTorch is a PyTorch extension that enables easy projection and un-projection of batches of points using a variety of linear and non-linear projection models.


## **Projection Models**

EyeTorch currently supports three camera projection models, each implemented as a Python class:


* `Pinhole`: The standard linear pinhole projection model
* `KannalaBrandt3`: a [generic camera model](https://ieeexplore.ieee.org/document/1642666), which is suitable for fish-eye lens cameras as well as conventional and wide-angle lens cameras
* `Fisheye624`: this is a python interface for the camera model we implemented in C++

## How to use EyeTorch

### 1. Open EyeTorch

```
$ python
>>> import eyetorch
>>> import torch
```



### 2. Select your projection model

 Use `camera = eyetorch.` projection model class `([475.0, 475.0, 320.0, 240.0]`) to select your projection model. The numbers in brackets represent the intrinsic parameters, focal_x, focal_y, principle_point_x and principle_point_y.


```
>>> camera = eyetorch.Pinhole([475.0, 475.0, 320.0, 240.0])
```



### 3.  Define the projection operator and un-projection operator

Each projection model provides two operators in addition to those it inherits from `torch.Tensor`. The projection operator maps points in 3D to points on a 2D image plane. The un-projection operator maps points on a 2D image plane to rays in 3D.

In this example 100 random 3D points will be projected.


#### Projection operator

```
points_3d = torch.randn(100, 3)
```



#### Un-projection operator

```
>>> points_2d = camera.project(points_3d)
```



## How EyeTorch works

To maximize simplicity, flexibility, and portability, EyeTorch projection models are  `torch.Tensor` objects. They can be used anywhere and in any way that a `torch.Tensor` object can be used, and are created in the same way `torch.Tensor` objects are created. PyTorch operators applied to an EyeTorch model will produce an EyeTorch model of the same type.

While EyeTorch projection model objects are simply `torch.Tensor` objects, they must maintain one invariant to be used for projection and un-projection of points: they must maintain a particular 'intrinsic size' in their last dimension. The intrinsic size is the number of parameters in the projection model. This is provided as a class attribute with the name `num_params`. This invariant is checked before applying the projection or un-projection operator but it is otherwise up to the user to maintain it.


##  **Tensor Size Requirements**

As discussed above, it is assumed that each EyeTorch object maintains its intrinsic size in the last dimension. Points tensor also have an intrinsic size that is expected and checked by the projection and un-projection operators: points passed to `project` are expected to have intrinsic size 3 in the last dimension and points passed to `unproject` are expected to have intrinsic size 2. Outputs of `project` have intrinsic size 2 and outputs of `unproject` have intrinsic size 3.

EyeTorch supports full broadcasting of both the camera parameters and the points tensors. For example, if a projection model has size `M`` x 1 x K` (where `K` is the model intrinsic size) and a points tensor has size `N x 3`, projecting the points will result in a tensor with size `M x N x 2` representing the projection of each point with each projection model.


## **Differentiability**

Eyetorch aims to provide differentiable operators wherever possible. However, it is not always possible to compute analytic partial derivatives for more complex non-linear projection models. Because of this, EyeTorch provides partial support for automatic differentiation in PyTorch.

The provided projection models all implement differentiation for the input points of projection and un-projection operators, and all implement differentiation with respect to the parameters for the projection operator.

Note: only the Pinhole model supports differentiation of un-projection for calibration parameters. This is represented in graphical form below, using the notation 'P(x; K)' to denote the projection of points x with parameters K and 'U(x; K)' to denote the un-projection of points x with parameters K.


|Model  |&#x2202;P / &#x2202;x |&#x2202;P / &#x2202;K |&#x2202;U / &#x2202;x |&#x2202;U / &#x2202;K |
|---    |---    |---    |---    |--- |
|`Pinhole` |YES |YES   |YES   |YES   |
|`KannalBrandt3` |YES |YES |YES |NO |
|`Fisheye624` |YES |YES |YES |NO |

For convenience, each projection model class defines boolean attributes named `project_differentiable_by_params` and `unproject_differentiable_by_params`  that enables you to dynamically query whether a particular model supports a particular type of differentiability.
