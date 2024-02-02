#pragma once

#include "calibmar/core/camera_models.h"

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class CameraModelWidget : public QWidget {
   public:
    CameraModelWidget(QWidget* parent = nullptr);
    CameraModelWidget(const std::function<void()> model_changed_callback, QWidget* parent = nullptr);

    CameraModelType CameraModel();
    void SetCameraModel(CameraModelType type);

   private:
    void SetCameraParametersLabel(int index);

    std::vector<std::tuple<CameraModelType, std::string, std::string>> camera_models_;
    QComboBox* camera_model_combobox_;
    QLabel* camera_parameters_label_;
    std::function<void()> model_changed_callback_;
  };
}
