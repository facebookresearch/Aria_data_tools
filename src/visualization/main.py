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
import sys
import time
from threading import Thread

import pyark.datatools as datatools

slam_camera_left_stream_id = datatools.dataprovider.StreamId(1201, 1)
slam_camera_right_stream_id = datatools.dataprovider.StreamId(1201, 2)
rgb_camera_stream_id = datatools.dataprovider.StreamId(214, 1)
stream_ids = [
    slam_camera_left_stream_id,
    slam_camera_right_stream_id,
    rgb_camera_stream_id,
]


def run_viewer(viewer):
    viewer.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualization example.")
    parser.add_argument(
        "--vrs_path", type=str, help="Path to the input VRS file or folder."
    )
    parser.add_argument(
        "--pose_path",
        type=str,
        help="Optional path to the input pose file.",
        default="",
    )
    args = parser.parse_args()
    data_provider = datatools.dataprovider.AriaVrsDataProvider()
    if data_provider.open(args.vrs_path, args.pose_path):
        print("Opened the VRS file successfully")
    else:
        print("Couldn't open the VRS file")
        sys.exit()
    # Stream players should be set after opening VRS file in AriaVrsDataProvider
    for stream_id in stream_ids:
        data_provider.setStreamPlayer(stream_id)
    fastest_nominal_rate_hz = data_provider.getFastestNominalRateHz()
    current_timestamp_sec = data_provider.getFirstTimestampSec()
    # Device model can be loaded now, VRS configuration record was read implicitly in getFastestNominalRateHz
    data_provider.loadDeviceModel()

    wait_time_sec = (1.0 / fastest_nominal_rate_hz) / 10
    viewer = datatools.visualization.AriaViewer(data_provider, 700, 800)
    viewer_thread = Thread(target=run_viewer, args=(viewer,))
    viewer_thread.start()

    # Setup viewer
    while not data_provider.atLastRecords():
        if viewer.isPlaying():
            for stream_id in stream_ids:
                if data_provider.tryFetchNextData(stream_id, current_timestamp_sec):
                    viewer.setCameraImageChanged(True, stream_id)
            current_timestamp_sec += wait_time_sec
            time.sleep(wait_time_sec)
    viewer_thread.join()
