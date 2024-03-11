#pragma once

#include "calibmar/core/image.h"
#include "calibmar/core/pixmap.h"

#include <Eigen/Core>

namespace calibmar {

  class TargetVisualizer {
   public:
    virtual ~TargetVisualizer() = default;

    virtual void DrawTargetOnImage(Pixmap& image, const Image& image_data) const = 0;
  };

  class ChessboardTargetVisualizer : public TargetVisualizer {
   public:
    ChessboardTargetVisualizer(int cols, int rows);

    void DrawTargetOnImage(Pixmap& image, const Image& image_data) const override;

    void DrawTargetPoseOnImage(Pixmap& image, const std::vector<Eigen::Vector2d>& target_pose_points) const;

   private:
    int cols_;
    int rows_;
  };

  class ArucoBoardTargetVisualizer : public TargetVisualizer {
   public:
    void DrawTargetOnImage(Pixmap& image, const Image& image_data) const override;
  };

  class Target3DTargetVisualizer : public TargetVisualizer {
   public:
    Target3DTargetVisualizer(bool contains_aruco, double scale_factor);

    void DrawTargetOnImage(Pixmap& image, const Image& image_data) const override;

   private:
    bool contains_aruco_;
    double scale_factor_;
  };
}
