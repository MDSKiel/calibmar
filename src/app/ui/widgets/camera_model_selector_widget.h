#pragma once

#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/core/report.h"
#include "ui/widgets/camera_model_widget.h"
#include "ui/widgets/initial_parameters_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class CameraModelSelectorWidget : public QGroupBox {
   public:
    CameraModelSelectorWidget(QWidget* parent = nullptr);

    CameraModelType CameraModel();
    void SetCameraModel(CameraModelType type);

    std::optional<std::vector<double>> InitialCameraParameters();
    void SetInitialCameraParameters(const std::optional<std::vector<double>>& parameters);

    bool Validate(std::string& error_message);

   private:
    CameraModelWidget* camera_model_;
    InitialParametersWidget* initial_parameters_;
  };
}
