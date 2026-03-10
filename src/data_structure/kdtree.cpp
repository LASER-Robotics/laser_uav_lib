#include "laser_uav_lib/data_structure/kdtree.hpp"

namespace laser_uav_lib
{
KDTree::KDTree() {
}

KDTree::KDTree(std::vector<geometry_msgs::msg::Point> points) {
  root = buildTree(points.begin(), points.end(), 0);
}

std::unique_ptr<KDNode> KDTree::buildTree(std::vector<geometry_msgs::msg::Point>::iterator begin, std::vector<geometry_msgs::msg::Point>::iterator end,
                                          int depth) {
  if (begin >= end)
    return nullptr;

  int  axis   = depth % 3;
  auto median = begin + (end - begin) / 2;

  std::nth_element(begin, median, end, [axis](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
    if (axis == 0)
      return a.x < b.x;
    if (axis == 1)
      return a.y < b.y;
    return a.z < b.z;
  });

  auto node   = std::make_unique<KDNode>();
  node->pt    = *median;
  node->left  = buildTree(begin, median, depth + 1);
  node->right = buildTree(median + 1, end, depth + 1);

  return node;
}

void KDTree::nearestSearch(KDNode* node, const geometry_msgs::msg::Point& target, int depth, double& best_dist_sq) const {
  if (!node)
    return;

  double d_sq =
      (node->pt.x - target.x) * (node->pt.x - target.x) + (node->pt.y - target.y) * (node->pt.y - target.y) + (node->pt.z - target.z) * (node->pt.z - target.z);
  if (d_sq < best_dist_sq) {
    best_dist_sq = d_sq;
  }

  int    axis = depth % 3;
  double diff = (axis == 0) ? (target.x - node->pt.x) : (axis == 1) ? (target.y - node->pt.y) : (target.z - node->pt.z);

  KDNode* first  = (diff < 0) ? node->left.get() : node->right.get();
  KDNode* second = (diff < 0) ? node->right.get() : node->left.get();

  nearestSearch(first, target, depth + 1, best_dist_sq);

  if (diff * diff < best_dist_sq) {
    nearestSearch(second, target, depth + 1, best_dist_sq);
  }
}

double KDTree::closestDistanceSquared(const geometry_msgs::msg::Point& target) const {
  double best_dist_sq = std::numeric_limits<double>::max();
  nearestSearch(root.get(), target, 0, best_dist_sq);
  return best_dist_sq;
}
}  // namespace laser_uav_lib
