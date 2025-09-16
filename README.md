# Laser UAV Lib

`laser_uav_lib` is a modular ROS 2 library designed for core UAV (Unmanned Aerial Vehicle) functionalities. Developed by the LASER Robotics Team, this library provides essential utilities for signal filtering, attitude representation, and basic Kalman filtering for drone applications.

## Key Features

- **Signal Filtering System**: Comprehensive IMU data filtering with configurable IIR (Infinite Impulse Response) and Notch filters to remove noise and vibrations from accelerometer and gyroscope readings.
- **Robust Attitude Conversion**: A powerful and flexible utility class to seamlessly convert between different attitude representations, including Quaternions (Eigen, `tf2`, `geometry_msgs`), Euler angles (intrinsic/extrinsic), and rotation matrices.
- **Extended Kalman Filter Framework**: A template-based EKF implementation that can be extended for specific state estimation applications.
- **Modular Design**: Each component is built as a separate library, allowing for selective integration and optimal performance.
- **Comprehensive Testing**: The library includes unit tests using GTest to ensure reliability and correctness of all components.

## Core Components

The library is organized into three main modules:

### 1. `filter`

A collection of signal processing tools for sensor data conditioning:

- **`ImuFilter`**: A comprehensive filter container that applies a sequence of configurable filters to IMU data (accelerometer and gyroscope).
- **`IIRFilter`**: A generic Infinite Impulse Response filter implementation for basic signal processing.
- **`NotchFilter`**: A specialized filter designed to eliminate noise at specific frequency bands, crucial for removing motor-induced vibrations.

### 2. `attitude_converter`

A versatile utility for handling 3D rotations and orientations:

- **Multiple Input Formats**: Supports construction from Euler angles (intrinsic/extrinsic), quaternions (Eigen, tf2, geometry_msgs), rotation matrices, and angle-axis representations.
- **Seamless Conversions**: Automatic type casting between different attitude representations.
- **Utility Functions**: Calculate heading, attitude rates, directional vectors, and perform attitude manipulations.
- **Exception Safety**: Comprehensive error handling for invalid inputs and mathematical edge cases.

### 3. `kalman_filter`

Basic Kalman filtering framework:

- **`EKF`**: A template-based Extended Kalman Filter implementation that can be customized for different state dimensions and measurement types.
- **Flexible Architecture**: Provides virtual functions for state transition models, measurement models, and Jacobian calculations that can be overridden by derived classes.

## Package Structure

```
laser_uav_lib/
├── include/laser_uav_lib/
│   ├── attitude_converter/
│   │   └── attitude_converter.hpp
│   ├── filter/
│   │   ├── imu_filter.hpp
│   │   ├── irr_filter.hpp
│   │   └── notch_filter.hpp
│   └── kalman_filter/
│       ├── kalman_filter.hpp
│       └── ekf/
│           └── ekf.hpp
├── src/
│   ├── attitude_converter/
│   ├── filter/
│   └── kalman_filter/
└── tests/
    ├── filter/
    └── ekf/
```

## Dependencies

Before building, ensure you have the following dependencies installed on your system:

- **ROS 2** (Humble or newer recommended)
- **Eigen3**: A C++ template library for linear algebra.
    ```bash
    sudo apt-get update
    sudo apt-get install libeigen3-dev
    ```


## Building and Testing

To build the library, clone the repository into your ROS 2 workspace and use `colcon`:

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

## Component Libraries

The package builds the following individual libraries that can be linked independently:

- `irr_filter`: IIR filter implementation
- `notch_filter`: Notch filter for frequency-specific noise removal
- `imu_filter`: Complete IMU filtering system
- `attitude_converter`: Attitude representation conversions
- `ekf`: Extended Kalman Filter framework

## How to Use

### Using Individual Components

To integrate specific components from `laser_uav_lib` into your ROS 2 project:

1. **Add as dependency in your `package.xml`:**
```xml
<depend>laser_uav_lib</depend>
```

2. **Link against desired libraries in your `CMakeLists.txt`:**
```cmake
find_package(laser_uav_lib REQUIRED)

# For attitude conversions
target_link_libraries(your_target attitude_converter)

# For IMU filtering
target_link_libraries(your_target imu_filter)

# For the complete library
target_link_libraries(your_target laser_uav_lib)
```

3. **Include headers in your code:**
```cpp
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>
#include <laser_uav_lib/filter/imu_filter.hpp>
#include <laser_uav_lib/kalman_filter/ekf/ekf.hpp>
```

### Example Usage

#### Attitude Conversion
```cpp
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

// Create from Euler angles
laser_uav_lib::AttitudeConverter attitude(0.1, 0.2, 0.3); // roll, pitch, yaw

// Convert to different representations
geometry_msgs::msg::Quaternion ros_quat = attitude;
Eigen::Quaterniond eigen_quat = attitude;
tf2::Quaternion tf2_quat = attitude;

// Get heading and directional vectors
double heading = attitude.getHeading();
auto forward_vector = attitude.getVectorX();
```

#### IMU Filtering
```cpp
#include <laser_uav_lib/filter/imu_filter.hpp>

// Configure filters
std::vector<double> iir_a = {1.0, -0.5};
std::vector<double> iir_b = {0.25, 0.25};
Eigen::MatrixXd notch_freqs(1, 1);
notch_freqs(0, 0) = 50.0; // Remove 50Hz noise

// Create filter
laser_uav_lib::ImuFilter filter(
    true, iir_a, iir_b,  // Accelerometer IIR enabled
    true, 100.0, notch_freqs, 5.0,  // Accelerometer notch filter
    true, iir_a, iir_b,  // Gyroscope IIR enabled
    true, 100.0, notch_freqs, 5.0   // Gyroscope notch filter
);

// Filter IMU data
laser_uav_lib::ImuData filtered_data = filter.filter(raw_imu_data);
```

## Contributing

When contributing to this library, please ensure:

1. All new features include corresponding unit tests
2. Code follows the existing style and documentation standards
3. Changes are backward compatible unless specifically noted
4. Update documentation to reflect any API changes

## License

This project is licensed under the Apache License 2.0 - see the package.xml file for details.

