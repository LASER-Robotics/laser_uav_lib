# KalmanFilter

`laser_uav_lib::KalmanFilter<States, Inputs, Measurements>` is the abstract interface for state estimators. It defines prediction, measurement correction and read access to the state and covariance. [EKF](../ekf/README.md) implements this interface.

Include [laser_uav_lib/kalman_filter/kalman_filter.hpp](../../include/laser_uav_lib/kalman_filter/kalman_filter.hpp). The interface is header-only; link the implementation used by the calling code.

## Dimensions and methods

The template arguments define the state-vector length, control-input length and measurement-vector length. Vectors and matrices use Eigen with `double` elements.

| Method | Input or result |
| --- | --- |
| `predict(u, dt)` | Control vector with `Inputs` elements and elapsed time in seconds. |
| `correct(z)` | Measurement vector with `Measurements` elements. |
| `get_state()` | Const reference to the estimated vector with `States` elements. |
| `get_covariance()` | Const reference to the `States × States` covariance matrix. |

A derived class implements all four methods. The interface does not define a vehicle model, noise values or a sensor layout.

[Library overview](../../README.md)
