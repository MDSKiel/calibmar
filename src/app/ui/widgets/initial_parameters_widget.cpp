#include "initial_parameters_widget.h"

namespace calibmar {

  InitialParametersWidget::InitialParametersWidget(QWidget* parent) : QWidget(parent) {
    parameters_edit_ = new QLineEdit(this);
    parameters_edit_->setPlaceholderText("Initial Parameters");
    parameters_checkbox_ = new QCheckBox(this);
    connect(parameters_checkbox_, &QCheckBox::stateChanged, this,
            [this](int state) { parameters_edit_->setEnabled(parameters_checkbox_->isChecked()); });
    parameters_edit_->setEnabled(parameters_checkbox_->isChecked());

    QHBoxLayout* parameters_layout = new QHBoxLayout(this);
    parameters_layout->setContentsMargins(0, 0, 0, 0);
    parameters_layout->addWidget(parameters_checkbox_);
    parameters_layout->addWidget(parameters_edit_);
  }

  void InitialParametersWidget::SetInitialParameters(const std::optional<std::string>& parameters) {
    parameters_checkbox_->setChecked(parameters.has_value());
    if (parameters.has_value()) {
      parameters_edit_->setText(QString::fromStdString(parameters.value()));
    }
    else {
      parameters_edit_->clear();
    }
  }

  std::optional<std::string> InitialParametersWidget::InitialParameters() {
    if (parameters_checkbox_->isChecked()) {
      return parameters_edit_->text().toStdString();
    }
    else {
      return {};
    }
  }
}
