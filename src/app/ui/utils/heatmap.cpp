#include "heatmap.h"
#include <opencv2/imgproc.hpp>

namespace calibmar::heatmap {
  void GenerateHeatmap(const std::vector<Image>& images, std::pair<int, int> image_size, Pixmap& target_pixmap,
                       double raster_factor) {
    int height = image_size.second;
    int width = image_size.first;

    Pixmap heatmap;
    heatmap.Data() = cv::Mat::zeros(height, width, CV_8UC1);

    for (const auto& image : images) {
      AddPointsToRawHeatmap(image.Points2D(), heatmap, raster_factor);
    }

    ApplyColorMapToRawHeatmap(heatmap, target_pixmap);
  }

  void AddPointsToRawHeatmap(const std::vector<Eigen::Vector2d>& points, Pixmap& heatmap, double raster_factor) {
    int height = heatmap.Height();
    int width = heatmap.Width();
    int block_size = std::max(width, height) / raster_factor;
    cv::Rect image_rect(0, 0, width, height);

    for (const auto& point2D : points) {
      cv::Rect block(point2D.x() - block_size / 2, point2D.y() - block_size / 2, block_size, block_size);
      block &= image_rect;
      if (!block.empty()) {
        heatmap.Data()(block) += 1;
      }
    }
  }

  void ApplyColorMapToRawHeatmap(const Pixmap& raw_heatmap, Pixmap& color_heatmap) {
    cv::Mat normalized(raw_heatmap.Data().size(), CV_8UC1);
    cv::normalize(raw_heatmap.Data(), normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(normalized, color_heatmap.Data(), cv::COLORMAP_SUMMER);
  }
}