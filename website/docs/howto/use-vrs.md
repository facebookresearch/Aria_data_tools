---
sidebar_position: 4
id: use-vrs
title: Getting to Know and Use VRS Data
---

# Getting to Know and Use VRS Data

Project Aria data is stored within [VRS](https://facebookresearch.github.io/vrs/) data containers. This means that standard VRS commands can be used to extract and use the datasets. VRS was installed as part of Aria Research Kit: Aria Data Tools.


## Instructions

Use the following commands to examine, extract and inspect data contained in any VRS file.


## Check the VRS file’s validity and integrity

The `check` command decodes every record in the VRS file and prints how many records were decoded successfully. It proves that the VRS file is correct at the VRS level. You can also compute a checksum and ensure you have the right one. For more information see [VRS File Validation](https://facebookresearch.github.io/vrs/docs/VrsCliTool#file-validation).


```
`$ vrs check <file.vrs>`
`$ vrs checksum <file.vrs>`
```


If the file is not valid it’s normally because there is missing data that could lead to invalid behavior with the tooling. All files in the Aria Pilot Dataset are valid, so if you encounter that issue with that dataset, re-downloading the data should resolve this issue.


## Extract image (jpg) or audio (wav) content to folders

Use the following commands to extract images or audio files.  Use the `--to <folder_path>` to specify a destination folder where the data will be extracted, or it will be added to the current working directory.


```
`$ vrs extract-images <file.vrs> --to <imagefolder>
``$ vrs extract-audio <file.vrs> --to <audiofolder>`
```


To extract RAW image files, use


```
$ vrs extract-images <file.vrs> --raw-images --to <imagefolder>
```



## Extract all content to folders and JSONs

This command lets you extract all images, audio, and metadata into files


```
 `$ vrs extract-all <file.vrs> --to <folder>`
```


The metadata is extracted into a single .jsons file that contains a succession of json messages, one per line. Each line corresponds to a single record, in timestamp order, so it is possible to parse it even if the number of records is huge. Saving all the data in a single file prevents saturating your disk with possibly millions of small files.

Once extracted, your file will look like this:


```
    ├── file.vrs
    ├── all_data
      * `NNNN-MM` folders: image folders, one folder per stream containing images.
      ├── 1201-1 # SLAM Left images
          ├── *.jpg
      ├── 1201-2 # SLAM Right images
          ├── *.jpg
      ├── 211-1  # Eye Tracking images
          ├── *.jpg
      ├── 214-1  # RGB (Color) Camera images
          ├── *.jpg
      ├── metadata.jsons
      └── ReadMe.md
```


For more information, see [VRS Data Extraction](https://facebookresearch.github.io/vrs/docs/VrsCliTool#data-extraction).


## Inspect how many data recordings there are by type

```
   `$ vrs <file.vrs> | grep "] records."`
```


Will get you a return like this:
|Number of data recordings| Type of sensor | first or second data stream - device type [[Stream ID number](https://facebookresearch.github.io/vrs/docs/FileStructure)]|


```
623 Eye Camera Class #1 - device/aria [211-1] records.
1244 RGB Camera Class #1 - device/aria [214-1] records.
729 Stereo Audio Class #1 - device/aria [231-1] records.
3101 Barometer Data Class #1 - device/aria [247-1] records.
65 Time Domain Mapping Class #1 - device/aria [285-1] records.
623 Camera Data (SLAM) #1 - device/aria [1201-1] records.
623 Camera Data (SLAM) #2 - device/aria [1201-2] records.
61965 IMU Data (SLAM) #1 - device/aria [1202-1] records.
50002 IMU Data (SLAM) #2 - device/aria [1202-2] records.
619 Magnetometer Data (SLAM) #1 - device/aria [1203-1] records.

```
