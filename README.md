# LASER UAV Library

This package provides **C++ classes** and **utilities** designed for core functionalities of the Laser UAV System (LUS). It serves as a foundational library for signal filtering, attitude representation, and Kalman filtering.

## Overview and Modules

The main objective of this package is to provide optimized, modular, and testable components that can be easily integrated into other ROS 2 nodes.

### Signal Filtering
-   **Description:** A comprehensive suite of signal processing tools designed to condition sensor data. It includes an **IMU Filter** container that chains configurable **IIR (Infinite Impulse Response)** and **Notch filters**. These are crucial for removing motor-induced vibrations and high-frequency noise from accelerometer and gyroscope readings before they reach the estimators or controllers.

-   **Provided Classes:**
    -   `ImuFilter`: Main interface for filtering 6-DOF IMU data.
    -   `NotchFilter`: Removes specific frequency bands (e.g., motor noise).
    -   `IIRFilter`: Generic low-pass/high-pass filter implementation.

-   **Code Example:**
    ```cpp
    #include <laser_uav_lib/filter/imu_filter.hpp>

    // Configure notch filter to remove 50Hz noise
    Eigen::MatrixXd notch_freqs(1, 1);
    notch_freqs(0, 0) = 50.0; 

    // Initialize filter (enabled flags, coefficients, sampling rates)
    laser_uav_lib::ImuFilter filter(
        true, iir_a, iir_b,              // Accelerometer IIR
        true, 100.0, notch_freqs, 5.0,   // Accelerometer Notch
        true, iir_a, iir_b,              // Gyroscope IIR
        true, 100.0, notch_freqs, 5.0    // Gyroscope Notch
    );

    auto clean_data = filter.filter(raw_imu_data);
    ```

### Attitude Converter
-   **Description:** A robust utility class meant to solve the common headache of converting between different 3D rotation representations in ROS 2. It supports seamless casting between **Quaternions** (Eigen, `tf2`, `geometry_msgs`), **Euler Angles** (Intrinsic/Extrinsic), and **Rotation Matrices**. It also provides utilities for calculating heading, directional vectors, and yaw rates.

-   **Key Features:**
    -   Automatic type casting between Eigen, TF2, and ROS messages.
    -   Safe handling of singularities (gimbal lock).
    -   Heading and vector extraction.

-   **Code Example:**
    ```cpp
    #include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

    // Initialize from Roll, Pitch, Yaw
    laser_uav_lib::AttitudeConverter state(0.1, 0.0, 1.57);

    // Get current heading
    double current_heading = state.getHeading();

    // Convert directly to ROS message or Eigen
    geometry_msgs::msg::Quaternion ros_msg = state;
    Eigen::Quaterniond eigen_q = state;
    ```

### Kalman Filter Framework
-   **Description:** A template-based **Extended Kalman Filter (EKF)** implementation. It provides a flexible architecture where state transition models, measurement models, and Jacobian calculations can be overridden. This allows for rapid development of custom state estimators for specific UAV applications without rewriting the core filter logic.

-   **Code Example:**
    ```cpp
    #include <laser_uav_lib/kalman_filter/ekf/ekf.hpp>

    // Define an EKF with 13 States, 4 Inputs, and 6 Measurements
    class MyDroneEstimator : public laser_uav_lib::EKF<13, 4, 6> {
        void state_transition_model(...) override {
            // Implement system dynamics here
        }
    };
    ```

## Integration Guide

To use this library in your ROS 2 package:

1.  **package.xml**: Add `<depend>laser_uav_lib</depend>`.
2.  **CMakeLists.txt**:
    ```cmake
    find_package(laser_uav_lib REQUIRED)
    target_link_libraries(your_target laser_uav_lib)
    ```