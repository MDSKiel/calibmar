#include "camera_model_selector_widget.h"
#include "ui/utils/parse_params.h"


#include <colmap/sensor/models.h>


namespace calibmar {

  CameraModelSelectorWidget::CameraModelSelectorWidget(QWidget* parent) : QGroupBox(parent) {

    setTitle("Camera model");

    camera_model_ = new CameraModelWidget(this);
    initial_parameters_ = new InitialParametersWidget(this);

    QVBoxLayout* camera_model_layout = new QVBoxLayout(this);
    camera_model_layout->addWidget(camera_model_);
    camera_model_layout->addWidget(initial_parameters_);
  }

  CameraModelType CameraModelSelectorWidget::CameraModel() {
    return camera_model_->CameraModel();
  }

  void CameraModelSelectorWidget::SetCameraModel(CameraModelType type) {
    camera_model_->SetCameraModel(type);
  }

  std::optional<std::vector<double>> CameraModelSelectorWidget::InitialCameraParameters() {
    std::optional<std::string> parameters_string = initial_parameters_->InitialParameters();
    std::vector<double> params;
    if (!parameters_string.has_value() || !TryParseParams(params, parameters_string.value())) {
      return {};
    }

    return params;
  }

  void CameraModelSelectorWidget::SetInitialCameraParameters(const std::optional<std::vector<double>>& parameters) {
    if (parameters.has_value()) {
      initial_parameters_->SetInitialParameters(colmap::VectorToCSV(parameters.value()));
    }
    else {
      initial_parameters_->SetInitialParameters({});
    }
  }

  bool CameraModelSelectorWidget::Validate(std::string& error_message) {
    CameraModelType camera_model = CameraModel();
    std::optional<std::string> parameters_string = initial_parameters_->InitialParameters();
    if (parameters_string.has_value()) {
      std::vector<double> params;
      if (!TryParseParams(params, parameters_string.value())) {
        error_message = "Invalid camera parameter format.";
        return false;
      }
      else if (params.size() != CameraModel::CameraModels().at(camera_model).num_params) {
        error_message = "Camera parameters dont match camera model.";
        return false;
      }
    }

    return true;
  }
}