#pragma once

#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/core/report.h"
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

    std::optional<std::string> InitialCameraParameters();
    void SetInitialCameraParameters(const std::optional<std::string>& parameters);

   private:
    void SetCameraParametersLabel(int index);

    std::vector<std::tuple<CameraModelType, std::string, std::string>> camera_models_;
    QComboBox* camera_model_combobox_;
    QLabel* camera_parameters_label_;
    InitialParametersWidget* initial_parameters_;
  };
}
