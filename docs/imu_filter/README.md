# ImuFilter

`laser_uav_lib::ImuFilter` applies separate filter chains to the accelerometer and gyroscope axes. Each chain runs its enabled notch filters first, followed by its IIR filter. Every axis has its own filter history.

Include [laser_uav_lib/filter/imu_filter.hpp](../../include/laser_uav_lib/filter/imu_filter.hpp) and link `imu_filter`.

## Input and output

The class accepts and returns `laser_uav_lib::ImuData`, which contains a timestamp, frame ID, orientation, acceleration, angular velocity and covariance arrays. Filtering changes the selected acceleration and angular-velocity fields. Other fields, including orientation and covariance, are copied unchanged.

| Method | Use |
| --- | --- |
| `filter(data)` | Filters both accelerometer and gyroscope measurements. |
| `filterAccelerometer(data)` | Filters acceleration and preserves the other fields. |
| `filterGyro(data)` | Filters angular velocity and preserves the other fields. |

Keep one instance per IMU stream. This class processes supplied samples; the calling node handles subscriptions, timestamps and the combination of separate sensor streams.

## Constructor settings

Accelerometer and gyroscope settings are supplied separately, in that order:

| Settings | Meaning |
| --- | --- |
| IIR enable flag, `a`, `b` | Enables IIR filtering with the supplied feedback and feedforward coefficients. |
| Notch enable flag | Enables the chain of notch filters. |
| Notch sampling rate | Input sampling rate in Hz. |
| Notch frequencies | Eigen matrix with one row and one column per frequency, in Hz. |
| Notch bandwidth | Bandwidth in Hz, shared by the notch filters in that chain. |

Use coefficients for the actual accelerometer and gyroscope sampling rates. See [IIRFilter](../irr_filter/README.md) and [NotchFilter](../notch_filter/README.md) for their coefficient and frequency conventions.

The ROS 2 wrapper and YAML parameters are documented in [laser_uav_imu_filter](https://github.com/LASER-Robotics/laser_uav_imu_filter).

[Library overview](../../README.md)
