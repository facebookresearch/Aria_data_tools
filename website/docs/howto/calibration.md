---
sidebar_position: 7
id: calibration
title: Using Aria Calibration Sensor Data
---

# Using Aria Calibration Sensor Data

Calibration data can be used to determine the 6DoF transformation between any pair of Project Aria's components.

Project Aria devices contain multiple types of sensors that are all calibrated when each device is manufactured. The calibration process derives intrinsic and extrinsic parameters (relative poses between sensors) between some sensors. This information is stored on every device and stamped on every VRS data file it records.

In Python, you can fetch this information from VRS and parse it into a structured representation using the following scripts:


```
`>>> import pyark
>>> reader = pyark.RecordFileReader()
>>> reader.openFile('./data/aria_unit_test_sequence_calib.vrs')
[ProgressLogger][INFO]: 26881.515: Opening diskfile file...0
>>> deviceModel = pyark.DeviceModel.fromJson(pyark.getCalibrationFromVrsFile(reader))`
```



## Sensors

Project Aria devices contain the following sensors:


* 1 x 120 degree FOV Rolling Shutter RGB camera
* 2 x 150 degree FOV Global Shutter mono cameras for SLAM & hand tracking
* 2 x 80 degree FOV Global Shutter mono cameras for Eye-tracking with IR illumination
* 2 x 1KHz IMU + Barometer and Magnetometer env. sensors
* 7 x 48 KHz spatial microphones
* GPS 1 Hz

For now, our Sensor Model and Calibration tooling gather data from cameras and IMUs.
See [2. Sensors and Measurements (howto/sensors-measurements.md)]([2. Sensors and Measurements (howto/sensors-measurements.md): CVPR Aria Data Tools Readme and Docusaurus documentation](https://fb.quip.com/c72qAWJmw4Ra#temp:C:cMG847a21451d8f4e6c85c18ad9d) for how different sensors are described in the tooling.


## Calibration

In computer vision, camera calibration is managed by two set of parameters:


* **Intrinsics**: Parameters defining how a 3D points project to the image plane (Focal, Principal Point, distortions coefficients, etc.)
* **Extrinsics**: Parameters defining where the camera is in space (Rotation|Translation)

### Extrinsics

Extrinsic parameters are represented as a SE(3) matrix, that contains a rotation part (SO(3)) as a unit quaternion and a translation part as an R3 coordinate. To manipulate transformations such as relative pose between sensors, or re-projecting a 3D points from one sensor to another we use the [Sophus library](https://github.com/strasdat/Sophus).
In the code and the documentation throughout this project, we use the following notation:


* `p_sensor` represents an R3 point in the local coordinate system of `sensor`. e.g. `p_slamLeft`.
* `T_sensor1_sensor2` represents a relative SE(3) transformation from `sensor2` frame to `sensor1` frame. An easy mnemonic is the chaining principle: `T_sensor1_sensor2 * T_sensor2_sensor3 * p_sensor3 = p_sensor1`

You can transform a 3d point from one sensor to the another one using the `transform()` operator:


```
`>>> import numpy as np
>>> p_slamLeft = np.array([3.0, 2.0, 1.0])
>>> p_imuRight = deviceModel.transform(p_slamLeft, 'camera-slam-left', 'imu-right')
>>> p_imuRight
array([ 3.33343274, -1.41484796,  1.20512771])
>>> deviceModel.transform(p_imuRight, 'imu-right', 'camera-slam-left')
array([3., 2., 1.]) # as you see we retrieve the initial 3D point`
```



### Intrinsics

Cameras can be configured to have a function that maps a 3D point in its local coordinate frame to the image pixel space. The parameters of this projection function are called the intrinsic parameter of a camera. All cameras on Project Aria devices are fisheye cameras. This meaning they are modeled by a spherical projection followed by additional distortion correction (rather than being modeled by a pinhole projection plus distortion).
For Project Aria devices, we use:


* [Kannala-Brandt model](https://ieeexplore.ieee.org/document/1642666) for eye tracking cameras
* FisheyeRadTanThinPrism model for SLAM and RGB cameras

You can perform the projection and unprojection operations using the following Python script:


```
`>>> p_slamLeft = np.array([3.0, 2.0, 1.0])
>>> uv_slamLeft = deviceModel.getCameraCalib('camera-slam-left').projectionModel.project(p_slamLeft)>>> uv_slamLeft
array([583.48105528, 411.98136675])
>>> deviceModel.getCameraCalib('camera-slam-left').projectionModel.unproject(uv_slamLeft)
array([3., 2., 1.])`
```


The IMU sensors use a linear rectification model for both accelerometers and gyroscopes to rectify an R3 point in its local coordinate system. The model includes a 3x3 rectification matrix A (correcting scale and non-orthogonality) and a 3x1 bias vector `b`.

To apply vibration rectification, use the Python scripts:


```
`>>> p_imuLeft = np.array([3.0, 2.0, 1.0])
>>> deviceModel.getImuCalib('imu-left').accel.rectify(p_imuLeft)
array([2.93735023, 2.02130446, 0.87514154])`
```


This rectification process applies this formula:


```
`p_real = A.inv() * (p_raw - b)`
```


When applied to accelerometer data, `p_raw` represents acceleration.  When applied to gyroscope data `p_raw`  represents angular velocity.
