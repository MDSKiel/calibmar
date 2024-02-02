#include "housing_selector_widget.h"

namespace calibmar {

  HousingSelectorWidget::HousingSelectorWidget(QWidget* parent) : QGroupBox(parent) {
    setTitle("Housing Interface");
    housing_parameters_label_ = new QLabel(this);
    housing_model_ = new HousingWidget(this);
    initial_parameters_edit_ = new QLineEdit(this);
    initial_parameters_edit_->setPlaceholderText("Initial Parameters");

    QVBoxLayout* camera_model_layout = new QVBoxLayout(this);
    camera_model_layout->addWidget(housing_model_);
    camera_model_layout->addWidget(housing_parameters_label_);
    camera_model_layout->addWidget(initial_parameters_edit_);
  }

  std::optional<std::pair<HousingInterfaceType, std::string>> HousingSelectorWidget::HousingOptions() {
    auto housing_model = housing_model_->HousingType();
    if (!housing_model.has_value()) {
      return {};
    }
    else {
      return std::make_pair(*housing_model, initial_parameters_edit_->text().toStdString());
    }
  }

  void HousingSelectorWidget::SetHousingOptions(const std::optional<std::pair<HousingInterfaceType, std::string>>& options) {
    if (options.has_value()) {
      housing_model_->SetHousingType(options->first);
    }
    else {
      housing_model_->SetHousingType({});
      initial_parameters_edit_->clear();
    }
  }
}