#include "common_calibration_options_widget.h"
#include "ui/utils/parse_params.h"
#include <colmap/util/misc.h>

namespace calibmar {
  CommonCalibrationOptionsWidget::CommonCalibrationOptionsWidget(QWidget* parent) : QWidget(parent) {
    // camera model
    camera_model_selector_ = new CameraModelSelectorWidget(this);

    // housing type
    housing_type_selector_ = new HousingSelectorWidget(this);

    // calibration target groupbox
    calibration_target_options_ = new CalibrationTargetOptionsWidget(this);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(camera_model_selector_);
    layout->addWidget(housing_type_selector_);
    layout->addWidget(calibration_target_options_);
    layout->setSizeConstraint(QLayout::SetMinimumSize);

    layout->setContentsMargins(0, 0, 0, 0);
  }

  bool CommonCalibrationOptionsWidget::Validate() {
    std::optional<std::pair<HousingInterfaceType, std::string>> housing = housing_type_selector_->HousingOptions();
    if (housing.has_value()) {
      std::vector<double> params;
      if (!TryParseParams(params, housing.value().second)) {
        QMessageBox::information(this, "Validation Error", "Invalid housing parameter format.");
        return false;
      }
      else if (params.size() != HousingInterface::HousingInterfaces().at(housing.value().first).num_params) {
        QMessageBox::information(this, "Validation Error", "Housing parameters dont match housing type.");
        return false;
      }
      else {
        // validated, take parameters
        housing_calibration_ = {housing.value().first, params};
      }
    }
    else {
      housing_calibration_ = {};
    }

    std::string message;
    if (!camera_model_selector_->Validate(message)) {
      QMessageBox::information(this, "Validation Error", QString::fromStdString(message));
      return false;
    }
    else {
      // validated
      initial_camera_parameters_ = camera_model_selector_->InitialCameraParameters();
    }

    if (housing_calibration_.has_value() && !initial_camera_parameters_.has_value()) {
      QMessageBox::information(this, "Validation Error", "Housing calibration requires valid camera parameters.");
      return false;
    }

    return true;
  }

  CommonCalibrationOptionsWidget::Options CommonCalibrationOptionsWidget::GetOptions() {
    Options options;
    options.calibration_target_options = calibration_target_options_->CalibrationTargetOptions();
    options.camera_model = camera_model_selector_->CameraModel();
    options.housing_options = housing_calibration_;
    options.initial_camera_parameters = initial_camera_parameters_;
    return options;
  }

  void CommonCalibrationOptionsWidget::SetOptions(Options options) {
    camera_model_selector_->SetCameraModel(options.camera_model);
    camera_model_selector_->SetInitialCameraParameters(options.initial_camera_parameters);
    if(options.housing_options.has_value()){
      housing_type_selector_->SetHousingOptions(std::make_pair(options.housing_options->first, colmap::VectorToCSV(options.housing_options->second)));
    }
    else{
      housing_type_selector_->SetHousingOptions({});
    }
    calibration_target_options_->SetCalibrationTargetOptions(options.calibration_target_options);    
  }

  void CommonCalibrationOptionsWidget::ForceArucoFor3DTarget(bool force) {
    calibration_target_options_->ForceArucoFor3DTarget(force);
  }
}