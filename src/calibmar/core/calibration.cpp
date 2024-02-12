#include <calibmar/core/calibration.h>

#include "calibration.h"
#include <iostream>

namespace calibmar {
  void Calibration::GetCorrespondences(std::vector<std::vector<Eigen::Vector2d>>& points2D,
                                       std::vector<std::vector<Eigen::Vector3d>>& points3D) {
    points2D.clear();
    points3D.clear();
    points2D.resize(images_.size());
    points3D.resize(images_.size());
    for (size_t i = 0; i < images_.size(); i++) {
      class Image& image = images_[i];
      points2D[i].reserve(image.Correspondences().size());
      points3D[i].reserve(image.Correspondences().size());
      for (const auto& corresponcence : image.Correspondences()) {
        points2D[i].push_back(image.Point2D(corresponcence.first));
        points3D[i].push_back(Point3D(corresponcence.second));
      }
    }
  }

  void Calibration::InitializeFromReconstruction(const colmap::Reconstruction& reconstruction) {
    if (reconstruction.Cameras().size() != 1) {
      throw std::runtime_error("Reconstruction must contain exactly one camera!");
    }

    points3D_.clear();
    images_.clear();

    camera_ = reconstruction.Cameras().begin()->second;

    for (const auto image_id : reconstruction.RegImageIds()) {
      const colmap::Image& rec_image = reconstruction.Image(image_id);
      if (rec_image.CameraId() != camera_.camera_id) {
        continue;
      }

      calibmar::Image image;
      std::vector<Eigen::Vector2d> points2D;

      size_t i = 0;
      for (const auto& point2D : rec_image.Points2D()) {
        if (!point2D.HasPoint3D()) {
          continue;
        }

        // emplace will only add if key does not exist
        points3D_.emplace(point2D.point3D_id, reconstruction.Point3D(point2D.point3D_id).xyz);
        image.SetPoint3DforPoint2D(point2D.point3D_id, i);
        points2D.push_back(point2D.xy);
        i++;
      }

      image.SetPoints2D(points2D);
      image.SetPose(rec_image.CamFromWorld());
      images_.push_back(image);
    }
  }
}
