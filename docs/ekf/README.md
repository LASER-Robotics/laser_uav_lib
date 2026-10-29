# EKF

`laser_uav_lib::EKF<States, Inputs, Measurements>` implements extended Kalman prediction and correction. A derived class supplies the system dynamics, measurement model and their Jacobians. The filter estimates the state and its covariance from control inputs and measurements.

Include [laser_uav_lib/kalman_filter/ekf/ekf.hpp](../../include/laser_uav_lib/kalman_filter/ekf/ekf.hpp) and link `ekf`. It implements the [KalmanFilter](../kalman_filter/README.md) interface.

## Define a model

The template dimensions specify the numbers of state, control and measurement elements. Override these four hooks in the derived class:

| Hook | Output to fill |
| --- | --- |
| `state_transition_model(x, u, dt, x_out)` | Predicted state after the control input and elapsed time. |
| `calculate_jacobian_F(x, u, dt, F_out)` | State-transition Jacobian. |
| `measurement_model(x, z_out)` | Measurement predicted from the state. |
| `calculate_jacobian_H(x, H_out)` | Measurement Jacobian. |

The default hooks have empty bodies. Implement them before calling `predict()` or `correct()`.

## Initialize and update

The derived class can initialize the protected state `x_`, covariance `P_`, process-noise matrix `Q_` and measurement-noise matrix `R_`. The constructor starts the state at zero, uses identity matrices for `P_`, `Q_` and `R_`, and sets the measurement Jacobian to zero. Set these values for your model and sensors.

Call `predict(u, dt)` as control inputs arrive, then `correct(z)` when a measurement is available. `get_state()` and `get_covariance()` return const references to the current estimate. Noise matrices are used as supplied; prediction does not automatically scale `Q_` by `dt`.

The compiled library explicitly instantiates `EKF<13, 4, 13>` and `EKF<13, Eigen::Dynamic, Eigen::Dynamic>`. Other template dimensions need a corresponding instantiation in the implementation. Dynamic-sized matrices must be sized by the derived filter before use.

Vehicle-specific estimator classes used by LUS are in `laser_uav_estimators`.

[Library overview](../../README.md)
