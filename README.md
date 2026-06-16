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
1. /main.hpp (CONFIG_PATH and FPGA_PATH)
2. /NiFpga_FPGA_CANBus_4module_v3_steering.h (NiFpga_FPGA_CANBus_4module_v3_steering_Bitfile)

## compiler
```bash
git clone https://github.com/Yatinghsu000627/fpga_driver.git
cd fpga_driver
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/install -DCMAKE_INSTALL_PREFIX=$HOME/corgi_ws/install -DOPENSSL_ROOT_DIR=$HOME/corgi_ws/install/ssl
make
```

## Directory Structure
    ```
    corgi_ws/
    ├── fpga_driver/
    │   ├── src/               # Source code for the driver
    │   ├── include/           # Header files
    │   ├── cmake/             # CMake configuration files
    │   ├── config/            # YAML configuration files
    │   ├── fpga_bitfile/      # FPGA bitfile
    │   ├── build/             # Build directory (generated after compilation)
    │   └── README.md          # Project documentation
    └── install/
    ```

---

## Changelog

### 2026-06-16
- **GET_STATE function code is now read-only.** `encodeRequestState` was reworked
  so requesting motor state issues a read-only `GET_STATE` FC instead of writing
  state, removing the unused write path from `can_motor`. (`src/can_motor.cpp`,
  `src/motor_fsm.cpp`, `include/can_motor.hpp`, `include/mode.hpp`)

### 2026-06-15
- **Motor feedback decoding fix.** Negated velocity and torque values when
  decoding motor feedback so reported direction matches the convention used by
  the FSM. (`src/can_motor.cpp`, `src/motor_fsm.cpp`)
- **Hall calibration mode-state handling.** Hardened the hall-calibration loop:
  the motor now correctly reports `MOTOR` mode during calibration, the motor-count
  limit in the calibration loop was corrected, the calibration timeout was
  increased, and mode-state mismatches now produce clearer error logging that
  prints the actual observed state. (`src/motor_fsm.cpp`)

### 2026-06-12
- **Hall calibration scope.** Calibration is limited to the R/L motors with staged
  support for the H motor; added a `hall_calibrate_requested` flag to manage
  calibration state and refined the calibration loop/logic. (`src/motor_fsm.cpp`,
  `src/robot_fsm.cpp`, `src/can_channel.cpp`, `src/leg_module.cpp`,
  `include/robot_fsm.hpp`, `include/can_channel.hpp`, `include/leg_module.hpp`)
- **Initialization sequencing.** Updated hall-calibration step numbers in
  `handleInit`, handle motor FSM switch failure during the set-zero step, and skip
  the `set_zero` case during init. (`src/robot_fsm.cpp`)

### 2026-06-09
- **Motor power-on boot wait.** Added a wait for motor power-on boot and updated
  the initialization steps accordingly. (`src/robot_fsm.cpp`, `include/robot_fsm.hpp`)
- **Mode `h` behavior.** Case `h` now skips hall calibration and enters motor mode
  directly. (`src/robot_fsm.cpp`)
