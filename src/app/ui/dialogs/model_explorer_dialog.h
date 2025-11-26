#pragma once

#include <QtCore>
#include <QtWidgets>

#include "calibmar/core/pixmap.h"
#include "colmap/scene/camera.h"
#include "ui/widgets/camera_model_widget.h"
#include "ui/widgets/housing_widget.h"
#include "ui/widgets/image_widget.h"

namespace calibmar {

  class ModelExplorerDialog : public QDialog {
   public:
    ModelExplorerDialog(QWidget* parent = nullptr);

    virtual void done(int r) override;

   private:
    void SetupCameraModelSliders();
    void SetupHousingModelSliders();
    void RenderLoop();

    std::unique_ptr<std::thread> worker_thread_;

    colmap::Camera camera_;
    std::unique_ptr<Pixmap> target_image_;
    ImageWidget* image_widget_;
    QGridLayout* common_sliders_side_layout_;
    QGridLayout* camera_sliders_side_layout_;
    CameraModelWidget* camera_model_;
    QGridLayout* housing_sliders_side_layout_;
    HousingWidget* housing_model_;
    std::atomic_bool camera_changed_;
    std::atomic_bool cancel_;
    std::mutex value_lock_;
    std::vector<double> rotations_;

    double distance_;
    double fov_;
    double checker_size_;
  };
}
