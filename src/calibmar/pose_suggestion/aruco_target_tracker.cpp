#include "target_tracker.h"

namespace calibmar {
  ArucoTargetTracker::ArucoTargetTracker(const std::pair<int, int>& image_size, double limit_percentage)
      : TargetTracker(image_size, limit_percentage) {}
  bool ArucoTargetTracker::CheckPointsMatch(const std::vector<Eigen::Vector2d>& points_a,
                                            const std::vector<Eigen::Vector2d>& points_b) {
    if (points_a.empty() || points_b.empty() || points_a.size() != points_b.size()) {
      return false;
    }

    for (size_t i = 0; i < points_a.size(); i++) {
      if (abs(points_a[i].x() - points_b[i].x()) > limits_xy_.first ||
          abs(points_a[i].y() - points_b[i].y()) > limits_xy_.second) {
        return false;
      }
    }

    return true;
  };
}