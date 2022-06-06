#pragma once

#include "calibmar/extractors/chessboard_extractor.h"

namespace calibmar {
  // TargetTracker compares sets of chessboard target points
  // and determines if they are stable or identical within a certain tolerance.
  class TargetTracker {
   public:
    TargetTracker(const std::pair<int, int>& columns_rows, const std::pair<int, int>& image_size, double limit_percentage = 0.03);

    // Update the current points. Points shape must adhere to columns_rows
    void Update(const std::vector<Eigen::Vector2d>& current_points);
    // True, if the edge corners of last and current points are within image_size * limit_percentage
    bool IsStable();
    // True, if stable and current points and target points are within image_size * limit_percentage
    bool TargetPointsReached();
    // Set target points. Points shape must adhere to columns_rows
    void SetTargetPoints(const std::vector<Eigen::Vector2d>& target_points);

   private:
    bool CheckPointsMatch(const std::vector<Eigen::Vector2d>& points_a, const std::vector<Eigen::Vector2d>& points_b);

    std::vector<Eigen::Vector2d> target_points_;
    std::vector<Eigen::Vector2d> last_points_;
    bool stable_;
    std::vector<size_t> outer_corner_idx_;
    std::pair<double, double> limits_xy_;
  };
}