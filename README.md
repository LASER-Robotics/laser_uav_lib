# LASER UAV Library

`laser_uav_lib` contains the C++ utilities shared by the LASER UAV System (LUS): signal filters, attitude conversions, Kalman filtering, error metrics and spatial searches.

Other packages link these components into their nodes. The library has no ROS topics, services, launch files or parameter YAML of its own.

## Build

From the workspace root:

```bash
colcon build --packages-up-to laser_uav_lib
source install/setup.bash
```

## Use in a ROS 2 package

Add `<depend>laser_uav_lib</depend>` to your package's `package.xml`. In `CMakeLists.txt`, find the package and link your existing target:

```cmake
find_package(laser_uav_lib REQUIRED)
target_link_libraries(your_target laser_uav_lib)
```

The `laser_uav_lib` target links all components. You can also link individual targets: `irr_filter`, `notch_filter`, `imu_filter`, `attitude_converter`, `ekf`, `rmse` and `kdtree`.

## Classes

| Class | Role |
| --- | --- |
| [IIRFilter](docs/irr_filter/README.md) | Filters scalar samples using feedback and feedforward coefficients. |
| [NotchFilter](docs/notch_filter/README.md) | Attenuates a selected frequency band. |
| [ImuFilter](docs/imu_filter/README.md) | Filters accelerometer and gyroscope measurements. |
| [AttitudeConverter](docs/attitude_converter/README.md) | Converts rotation representations and calculates heading and body axes. |
| [EulerAttitude](docs/euler_attitude/README.md) | Stores roll, pitch and yaw angles. |
| [Vector3Converter](docs/vector3_converter/README.md) | Converts three-dimensional vectors between Eigen, TF2 and ROS messages. |
| [KalmanFilter](docs/kalman_filter/README.md) | Defines the common interface for state estimators. |
| [EKF](docs/ekf/README.md) | Implements extended Kalman prediction and correction for a supplied model. |
| [RMSE](docs/rmse/README.md) | Measures estimated points' distance to a reference path. |
| [KDTree](docs/kdtree/README.md) | Finds the nearest reference point in three dimensions. |
