#include <calibmar/core/calibration.h>

#include <iostream>

namespace calibmar {
  void Calibration::GetCorrespondences(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                                       std::vector<std::vector<Eigen::Vector3d>>& points3D) {
    for (size_t i = 0; i < images_.size(); i++) {
      class Image& image = images_[i];
      points2D.push_back(std::vector<Eigen::Vector2d>());
      points3D.push_back(std::vector<Eigen::Vector3d>());

      for (const auto& corresponcence : image.Correspondences()) {
        points2D[i].push_back(image.Point2D(corresponcence.first));
        points3D[i].push_back(Point3D(corresponcence.second));
      }
    }
  }
}