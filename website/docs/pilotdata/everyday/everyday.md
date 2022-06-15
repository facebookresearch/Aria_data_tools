---
sidebar_position: 3
id: Everyday Activities
title: Everyday Activities File Structure
---

# Everyday Activities File Structure

The Everyday Activities dataset contains multiple activity sequences for up to four Project Aria device users. We created recordings for 1-4 Project Aria wearers, using scripts to represent all day activities with always on sensing.

Records for each location are stored in their own folder. Each location contains one or several script folders. The script folder contains a .csv file providing details about each sequence,  and a folder for each sequence. Each sequence has one or several recording folders. Each recording folder contains a folder for each dataset. Each derived dataset is a .csv file and the raw sensor data is stored in `recording.vrs`

Each location folder contains the following assets:

```
 Location_n
    ├── data_summary.csv
    ├──  script_1
        ├── seq_1
            ├── recording_001
              ├── eyetracking
              | └── et_in_rgb_stream.csv
              ├── location
              | └── trajectory.csv
              ├── recording.vrs
              ├── speech2text
              | ├── speech_aria_domain.csv
              │ └── speech.csv
              ├── synchronization
                └── timestamp_mapping.csv
            ├── recording_002
```

One recording folder is generated per Project Aria device wearer in the sequence.


The different files contain the following:

* `data_summary.csv` : details about all the scripts, sequences and recordings in this folder.
* `eyetracking` : contains the [Eye Gaze Data](/pilotdata/reprojected-gaze.mdx)
* `location` : contains the [Location Output Data](/pilotdata/location-output.mdx)
* `recording.vrs` :  contains Project Aria raw sensor data
* `speech2text` : contains the [Speech2Text Output Data](/pilotdata/speech2text.md)
* `synchronization` : contains the [Multi-Users Data](/pilotdata/timestamps.md)



**Table 1:** *`data_summary.csv` Structure*

|location |script_id |seq_id |recording_id |vrs |trajectory_file |eyetracking_file |speech2text_aria |speech2text_wav |synchronization |location_reference |contain_audio |stream_id |
|--- |--- |--- |--- |--- |--- |--- |--- |--- |--- |--- |--- |--- |
|location_n_indoor/outdoor |1-5 |1-8 |1-4 |script_n/seq_n/recording_n/recording.vrs |script_n/seq_n/recording_n/location/trajectory.csv |script_n/seq_n/recording_n/eyetracking/et_in_rgb_stream.csv |script_n/seq_n/recording_n/speech2text/speech_aria_domain.csv |script_n/seq_n/recording_n/speech2text/speech.csv |N/A if only one wearer |location_n_indoor/outdoor |TRUE/FALSE |Unique 15 digit number |

*Note:* [Recording Scripts](/pilotdata/everyday/scripts.md) provides details about what activities are recorded in each script and sequence.


## Project Aria Raw Sensor Data

See [Sensors and Measurements](/sensors-measurements.md) for information about how sensors are named and labelled in the tooling, coordinate systems, time and units of measurement.

See [How Project Aria Uses VRS](/aria-vrs.md) for information about how the raw data is stored.
