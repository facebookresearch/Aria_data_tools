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

import argparse

import numpy as np
import pyark


def getCalibStrFromFile(filePath):
    ext = filePath.split(".")[-1]
    if ext == "vrs":
        reader = pyark.RecordFileReader()
        reader.openFile(filePath)
        return pyark.getCalibrationFromVrsFile(reader)
    elif ext == "json":
        with open(filePath, "r") as f:
            return f.read()
    else:
        raise Exception(f"Unsupported file type: {filePath}!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="pyark example.")
    parser.add_argument(
        "file_path", type=str, help="Path to the input VRS or calib JSON file."
    )
    args = parser.parse_args()

    # Fetch and parse device calibration info from input file.
    calibStr = getCalibStrFromFile(args.file_path)
    device = pyark.DeviceModel.fromJson(calibStr)
    print(f"Cameras: {device.getCameraLabels()}")
    print(f"IMUs: {device.getImuLabels()}")
    print(f"Magnetometers: {device.getMagnetometerLabels()}")
    print(f"Barometers: {device.getBarometerLabels()}")
    print(f"Microphones: {device.getMicrophoneLabels()}")

    # Project and unproject points with a camera model.
    camLabel = "camera-slam-left"
    p_slamLeft = np.array([3.0, 2.0, 1.0])
    uv_slamLeft = device.getCameraCalib(camLabel).projectionModel.project(p_slamLeft)
    print(
        f"Projecting 3D point {p_slamLeft} to image space of {camLabel}: "
        + f"{uv_slamLeft}."
    )
    p_slamLeft_convertBack = device.getCameraCalib(camLabel).projectionModel.unproject(
        uv_slamLeft
    )
    print(
        f"Unprojecting 2D pixel {uv_slamLeft} to 3D space in "
        + f"the frame of {camLabel}: {p_slamLeft_convertBack}."
    )

    # Transform points between sensor frames.
    imuLabel = "imu-left"
    p_imuLeft = device.transform(p_slamLeft, camLabel, imuLabel)
    print(
        f"Transforming {p_slamLeft} from {camLabel} frame to {imuLabel} "
        + f"frame: {p_imuLeft}"
    )

    # Rectifying points with the IMU accelerometer model.
    p_imuLeft_rect = device.getImuCalib(imuLabel).accel.rectify(p_imuLeft)
    print(
        f"Point {p_imuLeft} is rectified by the accelerometer model "
        + f"of {imuLabel} as: {p_imuLeft_rect}"
    )
