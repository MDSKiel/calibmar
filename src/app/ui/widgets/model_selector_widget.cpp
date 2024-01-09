#include "ui/widgets/model_selector_widget.h"
#include "ui/utils/parse_params.h"

#include "model_selector_widget.h"
#include <colmap/sensor/models.h>

namespace {
  void InitializeCameraModels(std::vector<std::tuple<calibmar::CameraModelType, std::string, std::string>>& camera_models) {
    for (auto& [type, model] : calibmar::CameraModel::CameraModels()) {
      camera_models.push_back({type, model.friendly_name, model.params_info});
    }
  }
}

namespace calibmar {

  CameraModelSelectorWidget::CameraModelSelectorWidget(QWidget* parent) : QGroupBox(parent) {
    InitializeCameraModels(camera_models_);

    setTitle("Camera model");
    camera_parameters_label_ = new QLabel(this);
    camera_model_combobox_ = new QComboBox(this);
    for (auto const& tuple : camera_models_) {
      camera_model_combobox_->addItem(QString::fromStdString(std::get<1>(tuple)));
    }
    connect(camera_model_combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &CameraModelSelectorWidget::SetCameraParametersLabel);
    camera_model_combobox_->setCurrentIndex(0);
    SetCameraParametersLabel(0);

    initial_parameters_ = new InitialParametersWidget(this);

    QVBoxLayout* camera_model_layout = new QVBoxLayout(this);
    camera_model_layout->addWidget(camera_model_combobox_);
    camera_model_layout->addWidget(camera_parameters_label_);
    camera_model_layout->addWidget(initial_parameters_);
  }

  CameraModelType CameraModelSelectorWidget::CameraModel() {
    return std::get<0>(camera_models_[camera_model_combobox_->currentIndex()]);
  }

  void CameraModelSelectorWidget::SetCameraModel(CameraModelType type) {
    int index = 0;
    for (size_t i = 0; i < camera_models_.size(); i++) {
      if (std::get<0>(camera_models_[i]) == type) {
        index = i;
        break;
      }
    }
    camera_model_combobox_->setCurrentIndex(index);
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

  void CameraModelSelectorWidget::SetCameraParametersLabel(int index) {
    camera_parameters_label_->setText(QString::fromStdString("Model Parameters: " + std::get<2>(camera_models_[index])));
  }
}
