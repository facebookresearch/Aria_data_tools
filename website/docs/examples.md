---
sidebar_position: 3
id: examples
title: Examples
---

# Examples

The Aria Research Kit: Aria Data Tools provide python code and a C++ library to work with VRS files data.


## Retrieve and Read Data Using the Aria Data Provider

### Python

In this example, records are extracted and read for the Left SLAM Camera from recording.vrs.
For more information about the Aria Data provider go to [Aria Sensor Data: Aria Data Provider](howto/dataprovider.md)


#### 1. Open and select the file to read

```
$ python
>> import pyark.datatools as datatools
>> vrs_data_provider = datatools.dataprovider.AriaVrsDataProvider()
>> vrs_data_provider.openFile(‘recording.vrs’)
```



#### 2. Select which sensor information to extract

Either by using high-level API:


```
>> vrs_data_provider.setSlamCamera1Player()
```


or by using a StreamID:


```
>> slam_camera_recordable_type_id = 1201
>> slam_left_camera_instance_id = 1
>> slam_camera_left_stream_id = datatools.StreamId(slam_camera_recordable_type_id, slam_left_camera_instance_id)
>> vrs_data_provider.setStreamPlayer(slam_camera_left_stream_id)
```



#### 3.  Set whether to print data layouts (optional)

By default, data layouts are not printed while reading records. Set the verbosity to True to print data layouts and False to not print data layouts:


```
>> vrs_data_provider.setVerbose(True)
```



#### 4. Set how you want to read the data stream

All records in timestamp order


```
>> vrs_data_provider.readAllRecords()
4822.486 Camera Data (SLAM) #1 [1201-1]: jpg, 44338 bytes. # JPEG compressed image data size before decompression
...
4832.286 Camera Data (SLAM) #1 [1201-1]: jpg, 64148 bytes.
4832.386 Camera Data (SLAM) #1 [1201-1]: jpg, 64174 bytes.
```


Or read a single data record by timestamp:


```
>> vrs_data_provider.readDataRecordByTime(slam_camera_left_stream_id, someTimestamp)
```



#### 5. Access the data stream

```
>>> slam_left_camera_player = vrs_data_provider.getSlamLeftCameraPlayer()
>>> slam_left_camera_data_record = slam_left_camera_player.getDataRecord()
>>> slam_left_camera_data_record.captureTimestampNs
 4832385508212

>>> slam_left_camera_data = slam_left_camera_player.getData()
>>> pixel_frame = slam_left_camera_data.pixelFrame
>>> buffer = pixel_frame.getBuffer()
>>> len(buffer)
307200 # JPEG image data decompressed internally in AriaImageSensorPlayer
# equal to SLAM camera image width (640) * image height(480)

```



#### 6. Read the first configuration record of a stream:

```
>> vrs_data_provider.readFirstConfigurationRecord(slam_camera_left_stream_id)
```



#### 7. Load the device model

There are calibration strings for each image and motion stream. Reading the configuration record for any one of them will load the device model.


```
>>> slam_left_camera_stream_id = slam_left_camera_player.getStreamId()
>>> slam_left_camera_stream_id
<pyark.datatools.dataprovider.StreamId object at 0x7f955808c270>
>>> vrs_data_provider.readFirstConfigurationRecord(slam_left_camera_stream_id)
True
>>> vrs_data_provider.loadDeviceModel()
True
>>> device_model = vrs_data_provider.getDeviceModel()
>>> device_model
<pyark.datatools.sensors.DeviceModel object at 0x7f955808c2b0>
```



### C++

The following instructions show running sample usages of Aria VRS data provider for reading records of all streams in an Aria VRS file verbosely:


```
$ cd build/data_provider
$ ./read_all <vrs_path> # Read records of all streams verbosely
$ ./read_selected <vrs_path> # Read records of selected streams verbosely
```



## Visualizing Aria Sequences and Pre-Computed Camera Trajectory

The following instructions start running the visualization tool:


### Python

```
$ cd src/visualization
$ python3 main.py ${vrs_path} ${optional_pose_path}
```



### C++

```
$ cd build/visualization
$ ./aria_viewer ${vrs_path} ${optional_pose_path}
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

For more information, about this tool, go to [Visualizing Aria Sequences and Pre-Computer Camera Trajectory](howto/visualizing.md)
