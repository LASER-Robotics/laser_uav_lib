# KDTree

`laser_uav_lib::KDTree` stores three-dimensional points for nearest-neighbor searches. The [RMSE](../rmse/README.md) class uses it to find the closest reference point to each estimated position.

Include [laser_uav_lib/data_structure/kdtree.hpp](../../include/laser_uav_lib/data_structure/kdtree.hpp) and link `kdtree`.

## Use

```cpp
#include <laser_uav_lib/data_structure/kdtree.hpp>

geometry_msgs::msg::Point reference;
reference.x = 1.0;
laser_uav_lib::KDTree tree({reference});
geometry_msgs::msg::Point query;
query.x = 1.5;
double squared_distance = tree.closestDistanceSquared(query);
```

The constructor builds the tree from a vector of `geometry_msgs::msg::Point`. `closestDistanceSquared()` returns the squared Euclidean distance to the nearest stored point, not the point or its index. Apply a square root when an ordinary distance is needed.

Build a new tree when the reference point set changes. Queries must use the same coordinate frame and units as the stored points. An empty tree returns `std::numeric_limits<double>::max()` from `closestDistanceSquared()`.

[Library overview](../../README.md)
