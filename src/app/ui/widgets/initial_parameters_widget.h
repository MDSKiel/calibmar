#pragma once

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class InitialParametersWidget : public QWidget {
   public:
    InitialParametersWidget(QWidget* parent = nullptr, bool show_checkbox = true);

    void SetInitialParameters(const std::optional<std::string>& parameters);
    std::optional<std::string> InitialParameters();

    void SetChecked(bool checked);

   private:
    QLineEdit* parameters_edit_;
    QCheckBox* parameters_checkbox_;
  };
}
