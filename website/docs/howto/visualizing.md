---
sidebar_position: 6
id: visualizing
title: Visualizing Aria Sequences and Pre-Computed Camera Trajectory
---
# Visualizing Aria Sequences and Pre-Computed Camera Trajectory
The visualization tool allows users to visualize recordings from the SLAM and RGB cameras and see a 3D visualization of the pre-computed trajectory.


## How to use the visualization tool

The following instructions start running the visualization tool in C++:


```
$ cd build/visualization
$ ./aria_viewer ${vrs_path} ${optional_pose_path}
```


or in Python:


```
$ cd src/visualization
$ python3 main.py ${vrs_path} ${optional_pose_path}
```


Pose path is optional, if not provided visualization tool will try inferring the full path of `trajectory.csv` from the VRS path by using the Aria Pilot Dataset structure:


```
├── location
│   └── trajectory.csv
├── ${vrs_path}
└── ...
```


You will see something like the image below in the viewer after you press the **Play** button.
![img image of visualization tool](/img/docs/aria_viewer.png)

## Features

On the left you will see a menu bar and on the right a 4x4 grid. The grid shows the image streams of `camera-slam-left` (upper left), `camera-slam-right` (upper right), `camera-rgb` (lower left), and the 3D visualization of the pre-computed trajectory (lower right). Note that if `trajectory.csv` file is missing, nothing will be visualized in the lower right grid.

The menu bar’s features are:


* **LeftImg**: show/hide `camera-slam-left` image stream
* **RightImg**: show/hide `camera-slam-right` image stream
* **RgbImg**: show/hide `camera-rgb` image stream
* **LeftCam**: show/hide the 3D frustums of `camera-slam-left` in the trajectory visualization
* **RightCam**: show/hide the 3D frustums of `camera-slam-right` in the trajectory visualization
* **Rig**: show/hide the coordinate systems of `imu-left` in the trajectory visualization
* **Trajectory**: show/hide the trajectory as lines connecting the the poses of `imu-left`
* **camSparsity**: sparsity level of the visualized camera poses. “1” shows all the camera poses at the frame-rate of `camera-slam-left`

## Notes

* The playback speed mimics the real recording speed.
* The SLAM cameras and the RGB cameras may have different frame-rates.
* The provided poses in `trajectory.csv` have 1k Hz frequency and express the `imu-left` poses. The visualized poses are based on the timestamps of the `slam-left-cam` and are obtained by interpolating the provided 1k Hz poses. Please refer to `utils.cpp` in `data_provider` in this tool kit for details about the interpolation. The poses of different sensors are visualized using the calibration parameters between `imu-left` and the other sensors.

If you want to know more about accessing Project Aria data go to [Accessing Aria Sensor Data: Aria Data Provider](dataprovider.md)
