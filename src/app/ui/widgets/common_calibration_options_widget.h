#pragma once

#include "calibmar/core/report.h"
#include "ui/widgets/calibration_target_widget.h"
#include "ui/widgets/housing_selector_widget.h"
#include "ui/widgets/model_selector_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class CommonCalibrationOptionsWidget : public QWidget {
   public:
    CommonCalibrationOptionsWidget(QWidget* parent = nullptr);

    bool Validate();
    void SetImportedParameters(const ImportedParameters& parameters);

    CameraModelType CameraModel();
    std::optional<std::vector<double>> InitialCameraParameters();
    std::optional<std::pair<HousingInterfaceType, std::vector<double>>> HousingOptions();
    CalibrationTargetOptionsWidget::Options CalibrationTargetOptions();

    void ForceArucoFor3DTarget(bool force);
   private:
    CameraModelSelectorWidget* camera_model_selector_;
    HousingSelectorWidget* housing_type_selector_;
    CalibrationTargetOptionsWidget* calibration_target_options_;

    std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_calibration_;
    std::optional<std::vector<double>> initial_camera_parameters_;
  };
}