---
sidebar_position: 2
id: Install
title: Install
---
# Install

## What is it?

The Aria Research Kit: Aria Data Tools provides Python code and a C++ library to work with VRS files data and requires a C++17 compiler.

:::note

Aria Data Tools can be built and used in a container (Docker/Podman) or as a local build (CentOS/Debian/MacOS) on Unix distribution.

:::

## Get the code

Download the code by checking out this Github repository by running these commands.


```
$ git clone https://github.com/facebookresearch/Aria_data_tools.git
```



## Build (Container)
It’s easiest to install Aria Data Tools through [Docker](https://www.docker.com/resources/what-container/) or [Podman](https://podman.io/) containers. [Containers](https://en.wikipedia.org/wiki/OS-level_virtualization) allow developers to package an application together with libraries and other dependencies, allowing them to provide a self contained environment for running software services. Once Docker or Podman containers are set up, you can install Aria Data Tools with two commands.

* [About Podman](https://podman.io/getting-started/)
    * [Quick installation instructions](/FAQ.md)
* [Get started with Docker](https://www.docker.com/products/docker-desktop/)


The following terminal commands are given for Podman. If you are using Docker, replace Podman with Docker in the commands below.

Use these commands to install the Aria Data Tools, C++ library and python code.


```
$ podman build . -t aria_data_tools
$ podman run -it --volume <your_local_data>:/data aria_data_tools:latest
```

In order to forward X11 display between the container and the host machine for Unix machine, please use those instructions:
```
$ podman run --rm -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --security-opt label=type:container_runtime_t -it localhost/aria_data_tools:latest
```


## Dependencies

The following dependencies are used by Aria Data Tools and are automatically installed:


* VRS - https://github.com/facebookresearch/vrs
* FMT - https://github.com/fmtlib/fmt
* SOPHUS - https://github.com/strasdat/Sophus
* CEREAL - https://github.com/USCiLab/cereal
* PANGOLIN - https://github.com/stevenlovegrove/Pangolin
* FAST-CPP-CSV-PARSER - https://github.com/ben-strasser/fast-cpp-csv-parser
* EIGEN - https://gitlab.com/libeigen/eigen.git
* PYBIND11 - https://github.com/pybind/pybind11
* PYTORCH - https://github.com/pytorch/pytorch

## Test
Some unit tests are compiled by default with the CMake `BUILD_ARK_TESTS` preprocessor.

```
$ ctest -j8
```
