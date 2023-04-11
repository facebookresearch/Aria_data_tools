# Getting Started

The simplest way to build Aria Data Tools is either to use:
- [Pre-compiled](#Using-pre-compiled-package-(pip-install)) python package from PyPI (The Python Package Index)
- From source for
  - [native build](#Using-build-from-source)
  - [container build](#Container-(Build-and-run-in-isolation))

Requirements:
- C++17 compiler
- Python3 (>=3.8 recommended)


# Using pre-compiled package (pip install)

<details>
<summary> In a nutshell </summary>
<!-- empty line -->

```
pip install projectaria_tools

python
>>> import projectaria_tools
>>> dir(projectaria_tools)
['__doc__', '__loader__', '__name__', '__package__', '__spec__', 'dataprovider', 'mps_io', 'sensors']
>>> quit()
```

</details>

<br>

<details>
<summary> Using isolated/virtual environment </summary>
<!-- empty line -->

```
# Here’s an e2e example that:
# - creates a virtual environment in python3.9
# - installs `projectaria_tools` pkg and imports it

python3.9 -m venv projectaria_env
source ./projectaria_env/bin/activate
pip install projectaria_tools
python
>>> import projectaria_tools
>>> dir(projectaria_tools)
['__doc__', '__loader__', '__name__', '__package__', '__spec__', 'dataprovider', 'mps_io', 'sensors']
>>> quit()
deactivate
rm -rf ./projectaria_env
```

</details>

# Using build from source

We are detailing here instruction to install Aria Data Tools on Debian/Fedora/MacOS systems.
The build process is separated in 2 steps:
1. Installing 3rd party requirements
2.a Cloning, compiling Aria Data Tools [C++]
2.b Cloning, compiling Aria Data Tools [Python]

## 1. Installing 3rd party requirements

Pick your platform and follow along the provided instructions:

<details>
<summary> Debian/Ubuntu </summary>
<!-- empty line -->

```
# Install build essentials && python pip essentials
sudo apt-get install -y cmake git build-essential
sudo apt-get install -y libpython3-dev python3-pip
```

```
# Install VRS dependencies
sudo apt-get install -y libgtest-dev libgmock-dev \
    libfmt-dev  \
    libturbojpeg-dev libpng-dev \
    liblz4-dev libzstd-dev libxxhash-dev \
    libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-chrono-dev libboost-date-time-dev
```

```
# Installing cereal library (header only) to get the last version
cd /tmp
git clone https://github.com/USCiLab/cereal.git -b v1.3.2
cd cereal
cmake -DSKIP_PORTABILITY_TEST=1 -DJUST_INSTALL_CEREAL=ON .
sudo make -j2 install
rm -rf /tmp/cereal
```
</details>

<details>
<summary> Fedora </summary>
<!-- empty line -->

```
# Install build essentials
sudo dnf install -y git cmake gcc gcc-c++ make
```

```
# Install python (or use your system config)
sudo dnf install -y python3-devel; pip3 install --upgrade pip; pip3 install pybind11[global] numpy
```

```
# Install VRS dependencies
sudo dnf install -y gtest-devel gmock-devel glog-devel \
                 fmt-devel lz4-devel libzstd-devel xxhash-devel \
                 boost-devel libpng-devel libjpeg-turbo-devel turbojpeg-devel;
```

```
# Installing cereal library (header only) to get the last version
cd /tmp
git clone https://github.com/USCiLab/cereal.git -b v1.3.2
cd cereal
cmake -DSKIP_PORTABILITY_TEST=1 -DJUST_INSTALL_CEREAL=ON .
sudo make -j2 install
rm -rf /tmp/cereal
```
</details>


<details>
<summary> MacOS </summary>
<!-- empty line -->

```
# Install build essentials with Homebrew
brew install git cmake
```

```
# Install python (or use your system config)
brew install python3
```

```
# Install VRS dependencies
brew install boost fmt sophus cereal googletest glog lz4 zstd xxhash libpng jpeg-turbo
```
</details>

## 2.a Cloning, compiling Aria Data Tools [C++]

<details>
<summary> Debian/Ubuntu/Fedora/MacOS </summary>
<!-- empty line -->

```
# Clone the project
git clone https://github.com/facebookresearch/Aria_data_tools.git --recursive
mkdir build
cmake -DCMAKE_BUILD_TYPE=RELEASE -S ./Aria_data_tools/src -B build
cd build
make -j
```
</details>

## 2.b Cloning, compiling Aria Data Tools [Python]

<details>
<summary> Debian/Ubuntu/Fedora/MacOS </summary>
<!-- empty line -->

```
# Go to source folder & build/install python bindings
cd Aria_data_tools/src
pip3 install --global-option=build_ext --global-option="-j2" .;
```

```
# Testing successful install
python
>>> import projectaria.tools
>>> dir(projectaria.tools.datatools)
['__doc__', '__loader__', '__name__', '__package__', '__spec__', 'dataprovider', 'mpsIO', 'sensors']
```
</details>


# A note on Visualization samples
Visualization samples using Pangolin are optional and can be de/activated easily at the cmake configure step `-DBUILD_WITH_PANGOLIN=ON`. Here are the instructions to setup Pangolin and its internal dependencies.

## 1. Installing Pangolin (minimum dependencies)

<details>
<summary> Ubuntu/Debian </summary>
<!-- empty line -->

```
sudo apt-get install -y libeigen3-dev libglew-dev libgl1-mesa-dev

```
</details>

<details>
<summary> Fedora </summary>
<!-- empty line -->

```
sudo dnf install -y glew-devel eigen3
```
</details>

<details>
<summary> MacOS </summary>
<!-- empty line -->

```
brew install eigen glew
```
</details>

## 2. Clone, build and install Pangolin

<details>
<summary> Debian/Ubuntu/Fedora/MacOS </summary>
<!-- empty line -->

```
cd /tmp
git clone https://github.com/stevenlovegrove/Pangolin.git -b v0.8
# Package installed above should be enough,
# FYI you can also install all Pangolin REQUIRED dependencies by using
# "./Pangolin/scripts/install_prerequisites.sh required"
mkdir Pangolin_Build && cd Pangolin_Build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_EXAMPLES=OFF ../Pangolin/
sudo make -j install
```
</details>

## 3. Compiling Aria Data Tools with Pangolin

<details>
<summary> Debian/Ubuntu/Fedora/MacOS </summary>
<!-- empty line -->

```
# Cloning the project
git clone https://github.com/facebookresearch/Aria_data_tools.git --recursive
# Building the project
mkdir build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_WITH_PANGOLIN=ON  -S ./Aria_data_tools/src -B build
cd build
make -j
# Testing a visualization sample
./visualization/aria_viewer ../Aria_data_tools/data/aria_unit_test_sequence_calib.vrs
```
</details>

# Container (Build and run in isolation)

Here are the instruction to compile and run Aria Data Tools in isolation in a container. [Podman](https://podman.io/) or Docker can be used.

<details>
<summary> Debian/Ubuntu/Fedora/MacOS </summary>
<!-- empty line -->

```
git clone https://github.com/facebookresearch/Aria_data_tools.git --recursive

cd Aria_data_tools/src

podman build . -t aria_data_tools

podman run -it aria_data_tools
```
</details>

## Working with container

### X11 forwarding

- If you setup allow X11 forwarding you can see GUI elements from your container on your host machine (tested on Linux):

```
podman run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix localhost/aria_data_tools:latest
```

### Mapping volume to use HOST data in your container

- Adding volume(s) in your container allow you to quickly iterate on code or local data:

```
podman run -it --volume <SOME_PATH>/aria_data_tools/:/tmp/src_aria --volume '<SOME_PATH>Documents/TestingData':/tmp/data_test aria_data_tools
```
