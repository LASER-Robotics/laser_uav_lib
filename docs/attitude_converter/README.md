# AttitudeConverter

`laser_uav_lib::AttitudeConverter` converts between quaternions, rotation matrices and Euler angles. Controllers and estimators use it to exchange rotation representations and calculate heading or body-axis directions.

Include [laser_uav_lib/attitude_converter/attitude_converter.hpp](../../include/laser_uav_lib/attitude_converter/attitude_converter.hpp) and link `attitude_converter`.

## Use

```cpp
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

laser_uav_lib::AttitudeConverter attitude(0.1, 0.0, 1.57);
double heading = attitude.getHeading();
geometry_msgs::msg::Quaternion message = attitude;
Eigen::Quaterniond quaternion = attitude;
Eigen::Matrix3d rotation = attitude;
```

Angles are in radians. The roll/pitch/yaw constructor defaults to `laser_uav_lib::RPY_EXTRINSIC`; pass `RPY_INTRINSIC` explicitly when that is the intended convention. Constructors also accept Eigen, TF2 and ROS quaternion/matrix types, Eigen angle-axis values and [EulerAttitude](../euler_attitude/README.md).

## Methods

| Methods | Purpose |
| --- | --- |
| `getRoll()`, `getPitch()`, `getYaw()` | Returns individual Euler angles. |
| `getExtrinsicRPY()`, `getIntrinsicRPY()` | Returns a tuple of roll, pitch and yaw in the selected convention. |
| `getHeading()` | Returns the angle of the rotated body x-axis projected onto the reference xy-plane. |
| `getHeadingRate(body_rates)` | Calculates heading rate from body angular velocity. |
| `getYawRateIntrinsic(heading_rate)` | Converts heading rate to intrinsic yaw rate. |
| `getVectorX()`, `getVectorY()`, `getVectorZ()` | Returns rotated body-axis unit vectors as `Vector3Converter` values. |
| `setHeading()`, `setYaw()` | Returns an attitude with the requested heading or yaw. Assign the returned value to retain the change. |

Use finite, normalized quaternions and valid rotation matrices. Heading and rate calculations can throw exceptions at singular orientations, including when the body x-axis has no horizontal projection.

[Library overview](../../README.md)
