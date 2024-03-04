#include "target_tracker.h"

namespace calibmar {
  TargetTracker::TargetTracker(const std::pair<int, int>& image_size, double limit_percentage)
      : stable_(false), limits_xy_(image_size.first * limit_percentage, image_size.second * limit_percentage) {}

  void TargetTracker::Update(const std::vector<Eigen::Vector2d>& current_points) {
    if (!current_points.empty()) {
      stable_ = CheckPointsMatch(current_points, last_points_);
    }
    else {
      stable_ = false;
    }

    last_points_ = current_points;
  }

  bool TargetTracker::IsStable() {
    return stable_;
  }

  void TargetTracker::SetTargetPoints(const std::vector<Eigen::Vector2d>& target_points) {
    target_points_ = target_points;
  }

  bool TargetTracker::TargetPointsReached() {
    if (!stable_) {
      return false;
    }

    return CheckPointsMatch(last_points_, target_points_);
  }

  bool TargetTracker::CheckPointsMatch(const std::vector<Eigen::Vector2d>& points_a,
                                       const std::vector<Eigen::Vector2d>& points_b) {
    if (points_a.empty() || points_b.empty()) {
      return false;
    }

    for (size_t index : outer_corner_idx_) {
      if (abs(points_a[index].x() - points_b[index].x()) > limits_xy_.first ||
          abs(points_a[index].y() - points_b[index].y()) > limits_xy_.second) {
        return false;
      }
    }

    return true;
  }
}