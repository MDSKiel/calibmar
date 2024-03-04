#include "target_tracker.h"

namespace calibmar {
  ChessboardTargetTracker::ChessboardTargetTracker(const std::pair<int, int>& columns_rows, const std::pair<int, int>& image_size,
                                                   double limit_percentage)
      : TargetTracker(image_size, limit_percentage) {
    // opencv convention
    size_t columns_inner = columns_rows.first - 1;
    size_t rows_inner = columns_rows.second - 1;
    // top left, top right, bottom left, bottom right
    outer_corner_idx_ = {0, columns_inner - 1, (rows_inner - 1) * columns_inner,
                         (rows_inner - 1) * columns_inner + columns_inner - 1};
  }
}