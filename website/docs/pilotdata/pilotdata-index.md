---
sidebar_position: 1
id: pilotdata-index
title: Aria Pilot Dataset Overview
---
import useBaseUrl from '@docusaurus/useBaseUrl'

# Aria Pilot Dataset Overview

The Aria Pilot dataset is the first open dataset captured using [Project Aria](https://about.facebook.com/realitylabs/projectaria/), Meta’s research device used for accelerating machine perception and AI research, developed at Reality-Labs Research.

The dataset provides sequences collected with Project Aria devices a variety of egocentric scenarios, including cooking, exercising, playing games and spending time with friends, for researchers to engage with the challenges of always-on egocentric vision.

In addition to providing raw sensor data from Project Aria, the Pilot Dataset also contains derived results from machine perception services which provide additional context to the spatial-temporal reference frames, such as:

* Per-frame eye tracking
* Accurate 3D trajectories of users across multiple everyday activities in the same location
* Shared space-time information between multiple wearers
* Speech-to-text annotation



The dataset is extensive, providing:

* 143 recordings for Everyday Activities
* 16 Recordings for Desktop Activities
* Over 2.1 million images
* Over 7.5 accumulated hours

The dataset is split into two subsets:

* Everyday Activities - Multiple activity sequences where 1-2 users wearing Project Aria devices participate in scenarios to capture time synchronized data in a shared world location.
* Desktop Activities: Multiple object tracking sequences using one Project Aria device synchronized with a multi-view motion capture system

A further subset is planned for release in the near future that will include outdoor activities. This subset will also include data recorded using Sensor Profile 10, which includes GPS, WiFi and BT data.

Go to the [Project Aria website](https://about.facebook.com/realitylabs/projectaria/datasets) to access the Aria Pilot Dataset.

## Everyday Activities

<video width="950" controls>
  <source src={useBaseUrl('video/map_merge.mp4')} type="video/mp4"/>
  Your browser does not support the video tag.
</video>

*Shared 3D Global Trajectories for Multi-User Activities in the Same Locationn*


The main dataset contains multiple activity sequences for one to two Project Aria device wearers. Each wearer followed scripts that represented a typical scenario people might encounter throughout their day, which the wearers could use as prompts throughout their data collection.

* [Activities](/pilotdata/everyday/activities.md) provides details of the scenarios recorded and where specific activities are in a recording sequence
* [Scripts](/pilotdata/everyday/scripts.md) provides more details about each scenario
* [Sensor Profiles](/pilotdata/everyday/sensor-profiles.md) provides information about the two recording sensor profiles used to gather data
* [Everyday Activities File Structure](/pilotdata/everyday/everyday.md) provides information about how the data is structured and information about raw sensor data

In addition to the raw sensor data, we’ve provided derived meta-data for:

* High-frequency trajectory in 1KHz - [Location Output Data](/pilotdata/location-output.mdx)
* High-frequency time-synchronization between multiple users in the same activity - [Timestamps Mapping Data](/pilotdata/timestamps.md)
* Eye gaze reprojected gaze in the RGB camera stream - [Eye Gaze Data](/pilotdata/reprojected-gaze.mdx)
* Time-aligned speech to text - [Speech2Text Output Data](/pilotdata/speech2text.md)

The data has been gathered across five indoor locations. Data for each location is [stored in their own folder](/pilotdata/everyday/everyday.md).

## Desktop Activities

For this subset of the dataset a Project Aria wearer manipulated a set of objects on a desktop while being recorded by a multi-view motion capture system. The Project Aria device’s data is synchronized with the multi-view motion capture system to provide additional viewpoints and ground truth motion. Most objects were selected from the [YCB Object Benchmark](https://www.ycbbenchmarks.com/).

* [Desktop Activities Data Overview](/pilotdata/desk/desktop_overview.mdx) provides an overview of the dataset and the data structure.
* [Desktop Activities Capture Setup](/pilotdata/desk/desktop_setup.md) provides more information about the capture setup.


<video width="950" controls>
  <source src={useBaseUrl('video/desk_12-demo.mp4')} type="video/mp4"/>
  Your browser does not support the video tag.
</video>

**Figure 2:** *Object Sorting & Tidying Multi-View*


## How to Use the Dataset

The Aria Pilot Dataset has been optimized to work with Aria Research Kit: Aria Data Tools.

* [Install Aria Research Kit: Aria Data Tools](/install.md)
* [Aria Data Tools Examples](howto/examples.md)
* [Aria Data Tools Documentation](/overview.md)
* [Sensors and Measurements](/sensors-measurements.md)


You can also work with this data using standard VRS commands.

* [How Project Aria uses VRS](aria-vrs.md)
* [Getting to know and use VRS Data](use-vrs.md)


## Privacy

All sequences within the Aria Pilot Dataset were captured using fully consented actors in controlled environments. Bystanders and bystander vehicle data was strictly avoided when collecting data. For Desktop Activities, recordings the actor wore a mask. For Everyday Activities, faces were blurred prior to public release.

[View Meta's principles of responsible innovation](https://about.facebook.com/realitylabs/responsible-innovation-principles/)


## License

The Aria Pilot Dataset is released by Meta under the Dataset License Agreement.
