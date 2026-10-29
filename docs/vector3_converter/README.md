# Vector3Converter

`laser_uav_lib::Vector3Converter` converts three-dimensional vectors between Eigen, TF2 and ROS messages. It preserves the coordinates; it does not rotate the vector, change its frame or convert units.

Include [laser_uav_lib/attitude_converter/attitude_converter.hpp](../../include/laser_uav_lib/attitude_converter/attitude_converter.hpp) and link `attitude_converter`.

## Use

```cpp
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

laser_uav_lib::Vector3Converter vector(1.0, 2.0, 3.0);
Eigen::Vector3d eigen_vector = vector;
tf2::Vector3 tf_vector = vector;
geometry_msgs::msg::Vector3 message = vector;
```

Constructors accept three scalar coordinates, `Eigen::Vector3d`, `tf2::Vector3` or `geometry_msgs::msg::Vector3`. Assignment to one of these vector types invokes the corresponding conversion operator.

[AttitudeConverter](../attitude_converter/README.md) also returns this type from its body-axis getters.

[Library overview](../../README.md)
