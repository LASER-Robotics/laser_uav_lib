# RMSE

`laser_uav_lib::RMSE` measures the spatial distance between estimated points and a reference path. For each estimated point, it finds the nearest reference point through [KDTree](../kdtree/README.md), then calculates the root mean square distance and the standard deviation of the distances.

Include [laser_uav_lib/metrics/rmse.hpp](../../include/laser_uav_lib/metrics/rmse.hpp) and link `rmse`.

## Use

```cpp
#include <laser_uav_lib/metrics/rmse.hpp>

laser_uav_lib::RMSE metrics;
geometry_msgs::msg::Point reference;
reference.x = 1.0;
geometry_msgs::msg::Point estimated = reference;
estimated.x = 1.1;
metrics.pushReference(reference);
metrics.pushEstimated(estimated);
auto [rmse, standard_deviation] = metrics.calculate();
```

## Methods and interpretation

| Method | Purpose |
| --- | --- |
| `pushReference(point)` | Adds a reference point. |
| `pushEstimated(point)` | Adds an estimated point. |
| `calculate()` | Returns `{rmse, standard_deviation}`. |
| `reset()` | Clears both point sets for a new measurement. |

Both sets must be nonempty and have the same number of points. Otherwise `calculate()` returns `{-1.0, -1.0}`. Distances use the same units as the input coordinates.

Points are matched by spatial proximity, not by insertion order or timestamp. This measures distance to the reference path; it does not measure timing error along that path. Put both sets in the same coordinate frame before adding them.

[Library overview](../../README.md)
