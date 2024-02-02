#include "camera_model_widget.h"

namespace {
  void InitializeCameraModels(std::vector<std::tuple<calibmar::CameraModelType, std::string, std::string>>& camera_models) {
    for (auto& [type, model] : calibmar::CameraModel::CameraModels()) {
      camera_models.push_back({type, model.friendly_name, model.params_info});
    }
  }
}

namespace calibmar {

  CameraModelWidget::CameraModelWidget(QWidget* parent) : CameraModelWidget(nullptr, parent) {}

  CameraModelWidget::CameraModelWidget(const std::function<void()> model_changed_callback, QWidget* parent) {
    InitializeCameraModels(camera_models_);

    camera_parameters_label_ = new QLabel(this);
    camera_model_combobox_ = new QComboBox(this);
    for (auto const& tuple : camera_models_) {
      camera_model_combobox_->addItem(QString::fromStdString(std::get<1>(tuple)));
    }
    connect(camera_model_combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &CameraModelWidget::SetCameraParametersLabel);
    camera_model_combobox_->setCurrentIndex(0);
    SetCameraParametersLabel(0);

    QVBoxLayout* camera_model_layout = new QVBoxLayout(this);
    camera_model_layout->setContentsMargins(0, 0, 0, 0);
    camera_model_layout->addWidget(camera_model_combobox_);
    camera_model_layout->addWidget(camera_parameters_label_);

    // assign last to not call it during construction
    model_changed_callback_ = model_changed_callback;
  }

  CameraModelType CameraModelWidget::CameraModel() {
    return std::get<0>(camera_models_[camera_model_combobox_->currentIndex()]);
  }

  void CameraModelWidget::SetCameraModel(CameraModelType type) {
    int index = 0;
    for (size_t i = 0; i < camera_models_.size(); i++) {
      if (std::get<0>(camera_models_[i]) == type) {
        index = i;
        break;
      }
    }
    camera_model_combobox_->setCurrentIndex(index);
  }

  void CameraModelWidget::SetCameraParametersLabel(int index) {
    camera_parameters_label_->setText(QString::fromStdString("Model Parameters: " + std::get<2>(camera_models_[index])));

    if (model_changed_callback_) {
      model_changed_callback_();
    }
  }
}
