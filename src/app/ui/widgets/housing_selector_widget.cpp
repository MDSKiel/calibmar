#include "housing_selector_widget.h"

namespace calibmar {

  HousingSelectorWidget::HousingSelectorWidget(QWidget* parent) : QGroupBox(parent) {
    setTitle("Housing Interface");
    housing_model_ = new HousingWidget(this);
    initial_parameters_edit_ = new QLineEdit(this);
    initial_parameters_edit_->setPlaceholderText("Initial Parameters");

    QVBoxLayout* layout = new QVBoxLayout(this);

    layout->addWidget(housing_model_);
    layout->addWidget(initial_parameters_edit_);
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
      initial_parameters_edit_->setText(QString::fromStdString(options->second));
    }
    else {
      housing_model_->SetHousingType({});
      initial_parameters_edit_->clear();
    }
  }
}