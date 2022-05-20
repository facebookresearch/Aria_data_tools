---
sidebar_position: 2
id: Install
title: The Tool Kit & Installation
---
# The Tool Kit &  Installation

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

The following terminal commands are given for Docker. If you are using Podman, replace Docker with Podman in the commands below.

The following commands will install the Aria Data Tools, C++ library and python code.


```
$ docker build . -t aria_data_tools
$ docker run -it --volume <your_local_data>:/data aria_data_tools:latest
```



## Build and install (Local)

Aria Data Tools depend on multiple 3rd party system libraries and toolchains. We have provided installation scripts for CentOS, Debian and MacOS operating systems.


### Install dependencies

#### CentOS-family setup

Use these commands if you use CentOS-family linux such as Fedora, Redhat, CentOS or any other distribution using dnf.


```
$ cd src/scripts
$ sh ./install_deps_centos.sh
```



#### Debian-family setup

Use these commands if you use Debian-family linux  such as Ubuntu, Debian or any other distribution using apt-get.


```
$ cd src/scripts
$ sh ./install_deps_debian.sh
```



#### MacOS  setup

Use these commands to set up on MacOS.


```
$ cd src/scripts
$ sh ./install_deps_mac.sh
```



### Build the C++ and python libraries

Once Aria Data Tools is running, build the C++ library.


```
$ cd ../..
$ mkdir build
$ cd build
$ cmake ../src
$ make -j8
```


Build and install the Python code (provided as a [pip](https://pypi.org/project/pip/) package).


```
$ pip install --global-option=build_ext --global-option="-j8" .
```



### Dependencies

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
