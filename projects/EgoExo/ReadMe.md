# Get Started

<details>
<summary> 1. Checkout/Compilation </summary>
<!-- empty line -->

```
# Clone the project & assuming Pangolin is setup on your machine
git clone https://github.com/facebookresearch/Aria_data_tools.git --recursive
mkdir build
cmake -DCMAKE_BUILD_TYPE=RELEASE -S ./Aria_data_tools/projects/EgoExo -B build
cd build
make -j
```
</details>

<details>
<summary> 2. Running the Data Inspector </summary>
<!-- empty line -->

```
./egoExoViewer <Dataset_Path>
```
</details>

<details>
<summary> 3. Dataset documentation </summary>
<!-- empty line -->

A EgoExo dataset is compounded of required and optional files such as:
- 1 to N Aria trajectory files as closed_loop_trajectory<ID>.csv
- 0 to 1 point cloud file in csv or csv.gz format as global_points<.csv/.csv.gz>
- 0 to 1 GoPro cloud file in csv format as gopro_calibrations.csv
```
</details>
