# EulerAttitude

`laser_uav_lib::EulerAttitude` stores roll, pitch and yaw as three scalar values. It is a small value type used with [AttitudeConverter](../attitude_converter/README.md).

Include [laser_uav_lib/attitude_converter/attitude_converter.hpp](../../include/laser_uav_lib/attitude_converter/attitude_converter.hpp) and link `attitude_converter`.

## Use

```cpp
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

laser_uav_lib::EulerAttitude angles(0.1, 0.0, 1.57);
double roll = angles.roll();
double pitch = angles.pitch();
double yaw = angles.yaw();
laser_uav_lib::AttitudeConverter attitude(angles);
```

Supply angles in radians. The getters return the stored values unchanged. Constructing `AttitudeConverter` from this type uses the default extrinsic roll/pitch/yaw convention.

[Library overview](../../README.md)
