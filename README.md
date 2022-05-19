# Aria Data Tools

[![Continuous Integration Status](https://github.com/facebookresearch/Aria_data_tools/actions/workflows/build-and-test.yml/badge.svg?branch=main)](https://github.com/facebookresearch/Aria_data_tools/actions/workflows/build-and-test.yml)

## About

Project Aria makes open data and open tooling available to support researchers expand the horizons of Augmented Reality, Machine Learning and Artificial Intelligence by releasing the Aria Pilot Dataset and Aria Research Kit: Aria Data Tools.

### What are Aria Data Tools?

Aria Data Tools provides C++ and Python tools to interact with [Project Aria](https://about.facebook.com/realitylabs/projectaria/) data to:

* Read & visualize Project Aria sequences and sensor data
* Retrieve calibration data and interact with Aria camera models

### What is Project Aria?

[Project Aria](https://about.facebook.com/realitylabs/projectaria/) is a research device that collects first-person view (egocentric) data in safe and controlled environments. Aria sensors suite are stored in [VRS](https://facebookresearch.github.io/vrs/docs/Overview) files, with data from:

* 1 x 120 degree FOV Rolling Shutter RGB camera
* 2 x 150 degree FOV Global Shutter mono cameras for SLAM & hand tracking
* 2 x 80 degree FOV Global Shutter mono cameras for Eye-tracking with IR illumination
* 2 x 1KHz IMU + Barometer and Magnetometer env. sensors
* 7 x 48 KHz spatial microphones
* GPS 1 Hz

### What is the Aria Pilot Dataset?

The Aria Pilot Dataset provides data from a variety of egocentric scenarios, from hiking outdoors to cooking to time synced multi-person multi-Project Aria device scenarios. Datasets also include Project Aria data time synced with data from XSens motion capture suits and multi-view camera recording rigs. We believe these datasets can enable researchers to build and foster reproducible research on Computer Vision and AI/ML algorithms for scene perception, reconstruction and user surrounding understanding.

Go to XXXXXX to download the Aria Pilot Dataset.

### What can I do with this kind of data?

Some of the ways researchers use Project Aria data, sensor and visualization tools are to:

* Build SLAM technology for life-long and multi-device localization problems
    * Develop visual-inertial odometry (VIO) technology
    * Research non-visual based localization (such as [_TLIO_](https://arxiv.org/abs/2007.01867))
* Object detection and pose estimation
    * 3D Object Detection & reconstruction [_FRODO_](https://openaccess.thecvf.com/content_CVPR_2020/html/Runz_FroDO_From_Detections_to_3D_Objects_CVPR_2020_paper.html), [_ODAM_](https://arxiv.org/abs/2108.10165)
    * Lift object-based representations from sensor data
    * Reconstruction task
    * Provide neural rendering of objects and scenes
* Explore multi-modal learning using additional sensor data

## Getting Started

* Documentation [[Overview (index.md)](TBD)]
* Aria data tool - [[build and install](https://github.com/facebookresearch/Aria_data_tools/blob/main/BUILD.md)]
* Aria Pilot Dataset - [[WEBSITE link](https://fb.quip.com/2nFgAAuoYxfb)]
* Aria Pilot Dataset documentation [[1. Aria Pilot Dataset Overview](TBD)]
* How to Use the Tools [(howto/index.md)](TBD)

## How to Contribute

We welcome contributions! See [CONTRIBUTING](https://github.com/facebookresearch/Aria_data_tools/blob/main/CONTRIBUTING.md) for details on how to get started, and our [code of conduct](https://github.com/facebookresearch/Aria_data_tools/blob/main/CODE_OF_CONDUCT.md).

## Citation

* Aria CVPR paper
* Aria Open Dataset
* This repository

If you use the Aria Pilot Dataset in your research, please cite the following [technical report/paper](link) and/or

If you use the dataset or tools in GitHub, please consider starring ⭐ us and citing:


```
@article{xxxx,
  title =   {xxxx},
  author =  {xxxx},
  journal = {xxxx},
  year =    {xxx}
}
```

## Contact

[is there an e-mail we can provide?]

## License

Aria Data Tools are released by Meta under the [Apache 2.0 license](https://github.com/facebookresearch/vrs/blob/main/LICENSE).
