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
from PIL import Image
from projectaria_tools.dataprovider import AriaVrsDataProvider, StreamId


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        dest="vrs_path",
        type=str,
        required=True,
        help="path to vrs file",
    )
    return parser.parse_args()


# DataProvider samples
# Extract 10 RGB camera thumbnails from a VRS file
# Teachings:
# - How to initialize the AriaVrsDataProvider
# - How to register a given Player to the DataProvider
# - How to initialize a StreamId and query data records for it at a given timestamp
if __name__ == "__main__":
    args = parse_args()

    aria_data_provider: AriaVrsDataProvider = AriaVrsDataProvider()
    if not aria_data_provider.openFile(args.vrs_path):
        print(f"failed to open vrs: {args.vrs_path}")

    aria_data_provider.setRgbCameraPlayer()

    aria_data_provider.setVerbose(True)

    # from https://facebookresearch.github.io/Aria_data_tools/docs/sensors-measurements/
    rgb_camera_recordable_type_id = 214
    rgb_camera_instance_id = 1

    rgb_camera_stream_id = StreamId(
        rgb_camera_recordable_type_id, rgb_camera_instance_id
    )

    recording_start = aria_data_provider.getFirstTimestampSec()
    recording_end = aria_data_provider.getLastDataRecord(rgb_camera_stream_id).timestamp

    sample_count = 10
    sample_timestamps = np.linspace(recording_start, recording_end, sample_count)

    width = aria_data_provider.getImageWidth(rgb_camera_stream_id)
    height = aria_data_provider.getImageHeight(rgb_camera_stream_id)

    resize_ratio = 10
    big_image = new_image = Image.new(
        "RGB", (int(width * sample_count / resize_ratio), int(height / resize_ratio))
    )
    current_width = 0

    for sample in sample_timestamps:
        aria_data_provider.readDataRecordByTime(rgb_camera_stream_id, sample)

        rgb_player = aria_data_provider.getRgbCameraPlayer()
        img = rgb_player.getData()

        img_buf = img.pixelFrame.getBuffer()
        buffer_array = np.array(img_buf, dtype=np.uint8)
        image_array = buffer_array.reshape((height, width, 3))
        image = Image.fromarray(image_array)

        new_size = (
            int(image.size[0] / resize_ratio),
            int(image.size[1] / resize_ratio),
        )
        image = image.resize(new_size).rotate(-90)
        big_image.paste(image, (current_width, 0))
        current_width = int(current_width + width / resize_ratio)

    big_image.show()
