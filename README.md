# LASER UAV Library

`laser_uav_lib` contains the C++ utilities shared by the LASER UAV System (LUS): signal filters, attitude conversions, Kalman filtering, error metrics and a k-d tree.

Other packages link these components into their nodes. The library has no ROS topics, services, launch files or parameter YAML of its own.

## Components

| Component | Header | Role |
| --- | --- | --- |
| `IIRFilter` | `laser_uav_lib/filter/irr_filter.hpp` | Filters scalar samples using feedback and feedforward coefficients. The filename uses `irr`. |
| `NotchFilter` | `laser_uav_lib/filter/notch_filter.hpp` | Attenuates a selected frequency band. |
| `ImuFilter` | `laser_uav_lib/filter/imu_filter.hpp` | Applies IIR and notch filters to the accelerometer and gyroscope axes. |
| `AttitudeConverter` | `laser_uav_lib/attitude_converter/attitude_converter.hpp` | Converts between Euler angles, quaternions and rotation matrices, including Eigen, TF2 and ROS message types. |
| `EKF` | `laser_uav_lib/kalman_filter/ekf/ekf.hpp` | Extended Kalman filter template with prediction, correction and model/Jacobian hooks. |
| RMSE utilities | `laser_uav_lib/metrics/rmse.hpp` | Tracking-error statistics. |
| K-d tree | `laser_uav_lib/data_structure/kdtree.hpp` | Spatial searches over three-dimensional points. |

### Signal filters

Construct `IIRFilter` with coefficient vectors `a` and `b`, then call `iterate()` for each sample. `a` contains the feedback coefficients and `b` the feedforward coefficients. Choose coefficients for the actual sampling rate.

`ImuFilter` takes separate accelerometer and gyroscope settings: enable flags, IIR coefficients, notch sampling rates, frequencies and bandwidths. It accepts and returns `ImuData`; `filterAccelerometer()` and `filterGyro()` are available for separate streams. The ROS wrapper and its parameter descriptions are in `laser_uav_imu_filter`.

### Attitude conversion

```cpp
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

// Initialize from roll, pitch and yaw in radians.
laser_uav_lib::AttitudeConverter state(0.1, 0.0, 1.57);

double current_heading = state.getHeading();
geometry_msgs::msg::Quaternion ros_msg = state;
Eigen::Quaterniond eigen_q = state;
```

### Kalman filtering

`EKF<States, Inputs, Measurements>` provides `predict()`, `correct()`, `get_state()` and `get_covariance()`. A derived filter supplies its state-transition model, measurement model and Jacobians. The vehicle-specific estimators used by LUS are in `laser_uav_estimators`.

## Build and use

From the workspace root:

```bash
colcon build --packages-up-to laser_uav_lib
source install/setup.bash
```

Add `<depend>laser_uav_lib</depend>` to your package's `package.xml`. In `CMakeLists.txt`, find the package and link your existing target:

```cmake
find_package(laser_uav_lib REQUIRED)
target_link_libraries(your_target laser_uav_lib)
```

The exported `laser_uav_lib` target links all components. Individual targets include `irr_filter`, `notch_filter`, `imu_filter`, `attitude_converter`, `ekf`, `rmse` and `kdtree`.
