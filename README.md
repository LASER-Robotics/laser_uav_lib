# Laser UAV Lib

`laser_uav_lib` is a comprehensive ROS 2 library designed for core UAV (Unmanned Aerial Vehicle) functionalities, with a special focus on robust state estimation, signal filtering, and attitude representation. Developed by the Laser UAV Team, this library provides a modular and high-performance toolkit for advanced drone applications.

## Key Features

  - **Extended Kalman Filter (EKF) for Drones**: A sophisticated 13-state EKF that fuses multiple odometry sources (VIO, LIO, etc.) and IMU data to provide a reliable state estimate (position, velocity, and attitude).
  - **Advanced Signal Filtering**: Includes a chain of configurable filters for IMU data, featuring IIR (Infinite Impulse Response) and Notch filters to remove noise and vibrations from accelerometer and gyroscope readings.
  - **Robust Attitude Conversion**: A powerful and flexible utility class to seamlessly convert between different attitude representations, including Quaternions (Eigen, `tf2`, `geometry_msgs`), Euler angles (intrinsic/extrinsic), and rotation matrices.
  - **ROS 2 Lifecycle Management**: The main estimation node is implemented as a Lifecycle Node, allowing for controlled state transitions (configure, activate, deactivate) within a ROS 2 system.
  - **Dynamic Reconfiguration**: Allows for on-the-fly switching of the primary odometry source through a ROS 2 service call, enabling adaptation to different flight phases or sensor availability.
  - **Comprehensive Testing**: The library is equipped with a full suite of GTest-based unit tests to ensure the reliability and correctness of the filters and the EKF's dynamic model.

## Core Components

The library is organized into three main modules:

### 1\. `drone_ekf`

This is the core of the library. It provides a specialized Extended Kalman Filter that models the drone's dynamics using control inputs (motor thrusts) for the prediction step and corrects the state using a flexible `MeasurementPackage`. A key feature is the use of **automatic differentiation** via the `autodiff` library to calculate Jacobians, which simplifies implementation and reduces the risk of mathematical errors.

### 2\. `filter`

A collection of signal processing tools primarily for sensor data conditioning:

  - **`ImuFilter`**: A container that applies a sequence of filters to IMU data.
  - **`IIRFilter`**: A generic Infinite Impulse Response filter implementation.
  - **`NotchFilter`**: A filter designed to eliminate noise at specific frequency bands, which is crucial for removing vibrations caused by motors.

### 3\. `attitude_converter`

A versatile utility for handling 3D rotations. It simplifies the process of converting between different formats and performing calculations involving attitude, such as calculating heading or rates from quaternions.

## Dependencies

Before building, ensure you have the following dependencies installed on your system.

  - **ROS 2** (Humble or newer recommended)
  - **Eigen3**: A C++ template library for linear algebra.
    ```bash
    sudo apt-get update
    sudo apt-get install libeigen3-dev
    ```
  - **Autodiff**: A C++ library for automatic differentiation.
    ```bash
    # Enter the folder
    cd ~/git/
    
    # Clone the repository from GitHub
    git clone https://github.com/autodiff/autodiff.git
    
    # Enter the created directory
    cd autodiff
    
    # Create a separate build directory (best practice)
    mkdir build
    
    # Enter the build directory
    cd build
    
    # Configure the build with CMake
    cmake ..
    
    # Compile the code using multiple cores
    make -j
    
    # Install the library system-wide (requires admin privileges)
    sudo make install
    ```
## Building and Testing

To build the library and run the tests, clone the repository into your ROS 2 workspace and use `colcon`:

```bash
# In your ROS 2 workspace root (e.g., ~/ros2_ws)
colcon build --packages-select laser_uav_lib
```

To run the included tests and verify that all modules are working correctly:

```bash
# After sourcing your workspace (source install/setup.bash)
colcon test --packages-select laser_uav_lib
```

To view detailed test results:

```bash
colcon test-result --verbose
```

## How to Use

To integrate `laser_uav_lib` into your ROS 2 project, add it as a dependency in your `package.xml` and link against the desired libraries (`drone_ekf`, `filter`, `attitude_converter`) in your `CMakeLists.txt`.

