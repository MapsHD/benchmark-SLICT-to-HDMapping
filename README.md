# SLICT converter

## Intended use 

This small toolset allows to integrate SLAM solution provided by [slict](https://github.com/brytsknguyen/slict/) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 1 workspace that :
  - submodule to tested revision of SLICT
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependencies

```shell
sudo apt install -y nlohmann-json3-dev
sudo apt install python3-catkin-tools
sudo apt install libgflags-dev
sudo apt install libgoogle-glog-dev
```

## livox_ros_driver
```shell
https://github.com/Livox-SDK/livox_ros_driver
https://github.com/Livox-SDK/livox_ros_driver2
```

## Dependecies for Sophus and Ceres
```shell
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0.tar.gz
tar -xzf cmake-3.24.0.tar.gz
sudo mv cmake-3.24.0 /opt
cd /opt/cmake-3.24.0
./bootstrap --prefix=/opt/cmake-3.24
make -j$(nproc)
sudo make install

cd ~
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
git checkout 3.4.0
mkdir build && cd build
/opt/cmake-3.24/bin/cmake .. -DCMAKE_INSTALL_PREFIX=/opt/eigen-3.4
sudo make install
```

## Ceres and Sophus
```shell
cd ~
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver && git fetch --all --tags
git checkout tags/2.1.0
mkdir build && cd build
/opt/cmake-3.24/bin/cmake .. \
  -DEigen3_DIR=/opt/eigen-3.4/share/eigen3/cmake
make -j$(nproc)
sudo make install

cd ~
git clone https://github.com/strasdat/Sophus
cd Sophus
mkdir build && cd build
/opt/cmake-3.24/bin/cmake .. \
  -DEigen3_DIR=/opt/eigen-3.4/share/eigen3/cmake
make -j$(nproc)
sudo make install
```

## Building

Clone the repo
```shell
mkdir -p /test_ws/src
cd /test_ws/src
git clone https://github.com/marcinmatecki/SLICT-to-HDMapping.git --recursive
cd ..
catkin build
```

## Usage - data SLAM:

Prepare recorded bag with estimated odometry:

In first terminal record bag:
```shell
rosbag record /kfcloud /opt_odom
```

 start odometry:
```shell 
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch slict run_mcdviral.launch bag_file:={path_to_the_bag}
```

## Usage - conversion:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
rosrun slict-to-hdmapping listener <recorded_bag> <output_dir>
```

## Example:

Download the dataset from [NTU-VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)
For this example, download eee_03.

## Record the bag file:

```shell
rosbag record /kfcloud /opt_odom -O {your_directory_for_the_recorded_bag}
```

## GenZ Launch:

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
roslaunch slict run_ntuviral.launch bag_file:={path_to_the_bag}
```

## During the record (if you want to stop recording earlier) / after finishing the bag:

```shell
In the terminal where the ros record is, interrupt the recording by CTRL+C
Do it also in ros launch terminal by CTRL+C.
```

## Usage - Conversion (ROS bag to HDMapping, after recording stops):

```shell
cd /test_ws/
source ./devel/setup.sh # adjust to used shell
rosrun slict-to-hdmapping listener <recorded_bag> <output_dir>
```
