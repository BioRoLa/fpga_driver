# fpga_driver

## Overview
This repository provides a driver for FPGA-based systems. It leverages the `yaml-cpp` and `Eigen` libraries for YAML configuration parsing and linear algebra computations, respectively. And this is build on sbrio and cooperate with grpc.

---

## Prerequisites

Before building the project, ensure that the following dependencies are installed on your system:

### 1. **yaml-cpp**
Clone the `yaml-cpp` repository and follow the installation instructions:
```bash
cd install
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install
make -j16
sudo make install
```
### 2. **Eigen**
Clone the `Eigen` repository and follow the installation instructions:
```bash
cd install
git clone git@gitlab.com:libeigen/eigen.git
cd eigen
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install
make -j16
sudo make install
```
### 3. **grpc**
Install gRPC: https://grpc.io/docs/languages/cpp/quickstart/  

## change address
1. /fpga_server.hpp (CONFIG_PATH and FPGA_PATH)
2. /NiFpga_FPGA_CANBus_4module_v3_steering.h (NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile)

## compiler
```bash
git clone https://github.com/Yatinghsu000627/fpga_driver.git
cd fpga_driver
mkdir build && cd build
$ cmake .. -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install -DOPENSSL_ROOT_DIR=$HOME/corgi_ws/install/ssl
make -j16
```

## Directory Structure
    .
    fpga_driver/
    ├── src/               # Source code for the driver
    ├── include/           # Header files
    ├── cmake/             # CMake configuration files
    ├── build/             # Build directory (generated after compilation)
    └── README.md          # Project documentation
