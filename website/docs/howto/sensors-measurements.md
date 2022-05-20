---
sidebar_position: 3
id: sensors-measurements
title: Sensors and Measurements
---

# Sensors and Measurements

## Sensors

Aria sensors suite are stored in [VRS](https://facebookresearch.github.io/vrs/docs/Overview) files, with data from:


* 1 x 120 degree FOV Rolling Shutter RGB camera
* 2 x 150 degree FOV Global Shutter mono cameras for SLAM & hand tracking
* 2 x 80 degree FOV Global Shutter mono cameras for Eye-tracking with IR illumination
* 2 x 1KHz IMU + Barometer and Magnetometer env. sensors
* 7 x 48 KHz spatial microphones
* GPS 1 Hz

Each sensor is associated with an instance-invariant name. For example, the left SLAM camera is named as "camera-slam-left". The name set of supported sensors can be fetched in Python with the `deviceModel.getCameraLabels()` command:


```
>>> deviceModel.getCameraLabels()
['camera-et-left', 'camera-et-right', 'camera-rgb', 'camera-slam-left', 'camera-slam-right']>>> deviceModel.getImuLabels()
['imu-left', 'imu-right']`
```


:::note

Left/Right are relative to the left and right side of the glasses when the user is wearing the device. Another way to differentiate between the two sides is that the left SLAM camera is closer to the RGB camera.

:::

The following IDs are used to refer to specific sensors:

|StreamId    |Stream    |vrs::RecordableTypeId    |VRS Instance ID    |Calibration labels for coordinate transform    |DataProvider API    |
|---    |---    |---    |---    |---    |---    |
|211-1    |EyeTracking camera    |EyeCameraRecordableClass (211)    |1    |camera-et-left; camera-et-right    |getEyeCameraPlayer()    |
|214-1    |RGB camera    |RgbCameraRecordableClass (214)    |1    |`camera-rgb`    |getRgbCameraPlayer()    |
|231-1    |Microphones    |StereoAudioRecordableClass (231)    |1    |    |getAudioPlayer()    |
|247-1    |Barometer    |BarometerRecordableClass (247)    |1    |    |getBarometerPlayer()    |
|281-1    |GPS    |GpsRecordableClass (281)    |1    |    |getGpsPlayer()    |
|283-1    |Bluetooth beacon    |BluetoothBeaconRecordableClass (283)    |1    |    |getBluetoothBeaconPlayer()    |
|285-1    |Time domain    |TimeRecordableClass (285)    |1    |    |getTimeSyncPlayer()    |
|1201-1    |Slam Camera Left    |SlamCameraData (1201)    |1    |`camera-slam-left`    |getSlamLeftCameraPlayer()    |
|1201-2    |Slam Camera Right    |SlamCameraData (1201)    |2    |`camera-slam-right`    |getSlamRightCameraPlayer()    |
|1202-1    |IMU sensor 1    |SlamImuData (1202)    |1    |`imu-right`    |getImuRightPlayer()    |
|1202-2    |IMU sensor 2    |SlamImuData (1202)    |2    |`imu-left`    |getImuLeftPlayer()    |
|1203-1    |Magnetometer    |SlamMagnetometerData (1203)    |1    |    |getMagnetometerPlayer()    |


## Coordinate Systems

Applications like stereo vision and navigation usually handle 2D and 3D points in different spaces and transformations need to be conducted between them. With Project Aria data, we attach a local R3 coordinate frame to each sensor. The local coordinate frames are:


* Camera: The origin is at the optical axis in the central pupil plane. When facing the camera, X+ points to right, Y+ points down and Z+ points outwards.
* IMU: The accelerometer is picked as the origin of the IMU frame. When facing the chip, X+ points to right, Y+ points to up and Z+ points outwards.

## Time

Every signal (or Record in VRS terms) collected by sensors is stamped with a timestamp from a common clock. For Project Aria data this is usually the board clock. All records are sorted in monotonically increasing order in a VRS file. We use the following sensor-specific conventions on timestamps:


* For all cameras, the timestamp of a frame is the center exposure time, a.k.a. the middle point of exposure interval.
* RGB camera uses a rolling shutter, with a readout time of 5ms(low-res)/15ms(high-res) from top to bottom. The recorded timestamp of an RGB frame is the center exposure timestamp of the center row of the image. SLAM and eyetracking cameras use global shutters and do not have this issue.
* For IMUs, accelerometers and gyroscopes may have a time offset from the board clock. This is calibrated and stored in the JSON as `TimeOffsetSec_Device_Gyro` and `TimeOffsetSec_Device_Accel`.

## Units of Measurement

The units for numerical values in the code and documentation are:


* Coordinates, location and distance in world space: meters (m)
* Coordinates in image space: pixels
* Timestamp and time intervals: seconds (s)
* Angles: radians (rad)
* Acceleration: m/s^2
* Rotation: rad/s
