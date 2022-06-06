#include "ui/widgets/housing_selector_widget.h"

namespace {
  void InitializeHousingModels(
      std::vector<std::tuple<calibmar::HousingInterfaceType, std::string, std::string>>& housing_models) {
    for (auto& [type, housing] : calibmar::HousingInterface::HousingInterfaces()) {
      housing_models.push_back({type, housing.friendly_name, housing.params_info});
    }
  }
}

namespace calibmar {

  HousingSelectorWidget::HousingSelectorWidget(QWidget* parent) : QGroupBox(parent) {
    InitializeHousingModels(housing_models_);

    setTitle("Housing Interface");
    housing_parameters_label_ = new QLabel(this);
    housing_model_combobox_ = new QComboBox(this);
    housing_model_combobox_->addItem("None");
    for (auto const& tuple : housing_models_) {
      housing_model_combobox_->addItem(QString::fromStdString(std::get<1>(tuple)));
    }
    connect(housing_model_combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &HousingSelectorWidget::SetCameraParametersLabel);
    housing_model_combobox_->setCurrentIndex(0);
    initial_parameters_edit_ = new QLineEdit(this);
    initial_parameters_edit_->setPlaceholderText("Initial Parameters");
    SetCameraParametersLabel(0);

    QVBoxLayout* camera_model_layout = new QVBoxLayout(this);
    camera_model_layout->addWidget(housing_model_combobox_);
    camera_model_layout->addWidget(housing_parameters_label_);
    camera_model_layout->addWidget(initial_parameters_edit_);
  }

  std::optional<std::pair<HousingInterfaceType, std::string>> HousingSelectorWidget::HousingOptions() {
    int idx = housing_model_combobox_->currentIndex();
    if (idx == 0) {
      // index 0 is "None"
      return {};
    }
    else {
      return std::make_pair(std::get<0>(housing_models_[idx - 1]), initial_parameters_edit_->text().toStdString());
    }
  }

  void HousingSelectorWidget::SetHousingOptions(const std::optional<std::pair<HousingInterfaceType, std::string>>& options) {
    if (options.has_value()) {
      int index = 0;
      for (size_t i = 0; i < housing_models_.size(); i++) {
        if (std::get<0>(housing_models_[i]) == options.value().first) {
          index = i;
          break;
        }
      }
      // accout for NONE offset
      housing_model_combobox_->setCurrentIndex(index + 1);
      initial_parameters_edit_->setText(QString::fromStdString(options.value().second));
    }
    else {
      // index 0 is "None"
      housing_model_combobox_->setCurrentIndex(0);
      initial_parameters_edit_->clear();
    }
  }

  void HousingSelectorWidget::SetCameraParametersLabel(int index) {
    if (index == 0) {
      // index 0 is "None"
      housing_parameters_label_->setVisible(false);
      initial_parameters_edit_->setEnabled(false);
    }
    else {
      housing_parameters_label_->setVisible(true);
      initial_parameters_edit_->setEnabled(true);
      housing_parameters_label_->setText(QString::fromStdString("Parameters: " + std::get<2>(housing_models_[index - 1])));
    }
  }
}