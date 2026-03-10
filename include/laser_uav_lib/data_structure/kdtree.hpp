#ifndef LASER_UAV_LIB__KDTREE_HPP
#define LASER_UAV_LIB__KDTREE_HPP

#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

#include <geometry_msgs/msg/point.hpp>

namespace laser_uav_lib
{

struct KDNode
{
  geometry_msgs::msg::Point pt;
  std::unique_ptr<KDNode>   left;
  std::unique_ptr<KDNode>   right;
};

class KDTree {
private:
  std::unique_ptr<KDNode> root;
  std::unique_ptr<KDNode> buildTree(std::vector<geometry_msgs::msg::Point>::iterator begin, std::vector<geometry_msgs::msg::Point>::iterator end, int depth);
  void                    nearestSearch(KDNode* node, const geometry_msgs::msg::Point& target, int depth, double& best_dist_sq) const;

public:
  KDTree();
  KDTree(std::vector<geometry_msgs::msg::Point> points);
  double closestDistanceSquared(const geometry_msgs::msg::Point& target) const;
};
}  // namespace laser_uav_lib
#endif
