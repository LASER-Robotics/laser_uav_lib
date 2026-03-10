#include <laser_uav_lib/metrics/rmse.hpp>
#include <cmath>

namespace laser_uav_lib
{
RMSE::RMSE() {
}

void RMSE::pushReference(const geometry_msgs::msg::Point reference_point) {
  reference_.push_back(reference_point);
}

void RMSE::pushEstimated(const geometry_msgs::msg::Point estimated_point) {
  estimated_.push_back(estimated_point);
}

void RMSE::reset() {
  reference_.clear();
  estimated_.clear();
}

std::pair<double, double> RMSE::calculate() {
  if (estimated_.empty() || estimated_.size() != reference_.size()) {
    std::cout << "[RMSE] Metric calculation failed: reference and estimated sets have different sizes." << std::endl;
    return {-1.0, -1.0};
  }

  tree = laser_uav_lib::KDTree(reference_);

  std::vector<double> distances;
  distances.reserve(estimated_.size());

  double sum_dist    = 0.0;
  double sum_sq_dist = 0.0;

  for (const auto& est_pt : estimated_) {
    double min_dist_sq = tree.closestDistanceSquared(est_pt);
    double min_dist    = std::sqrt(min_dist_sq);

    distances.push_back(min_dist);
    sum_dist += min_dist;
    sum_sq_dist += min_dist_sq;
  }

  size_t n = distances.size();

  double rmse_val = std::sqrt(sum_sq_dist / n);

  double mean_dist = sum_dist / n;
  double variance  = 0.0;
  for (double d : distances) {
    variance += (d - mean_dist) * (d - mean_dist);
  }
  double std_dev_val = std::sqrt(variance / n);

  return {rmse_val, std_dev_val};
}
}  // namespace laser_uav_lib
