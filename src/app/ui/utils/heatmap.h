#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/core/pixmap.h"

namespace calibmar::heatmap {

  // Generate a calibration heatmap from the calibration and store it in the target pixmap. Pixmap data is only allocated if needed.
  // The raster factor determines the feature point block size as max(width, height) / raster_factor.
  void GenerateHeatmap(const std::vector<Image>& images, std::pair<int, int> image_size, Pixmap& target_pixmap, double raster_factor = 25);

  // Add points to a already allocated (correctly sized) heatmap.
  void AddPointsToRawHeatmap(const std::vector<Eigen::Vector2d>& points, Pixmap& heatmap, double raster_factor = 25);

  // Create a colored and normalized heatmap from a raw heatmap
  void ApplyColorMapToRawHeatmap(const Pixmap& raw_heatmap, Pixmap& color_heatmap);
}