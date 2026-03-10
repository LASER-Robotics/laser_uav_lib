#ifndef LASER_UAV_LIB__RMSE_HPP
#define LASER_UAV_LIB__RMSE_HPP

#include <vector>
#include <utility>
#include <iostream>
#include <laser_uav_lib/data_structure/kdtree.hpp>

namespace laser_uav_lib
{
class RMSE {
public:
  RMSE();
  void                      pushReference(const geometry_msgs::msg::Point estimated_point);
  void                      pushEstimated(const geometry_msgs::msg::Point estimated_point);
  void                      reset();
  std::pair<double, double> calculate();

private:
  laser_uav_lib::KDTree                                 tree;
  std::vector<geometry_msgs::msg::Point> reference_;
  std::vector<geometry_msgs::msg::Point> estimated_;
};
}  // namespace laser_uav_lib

#endif
