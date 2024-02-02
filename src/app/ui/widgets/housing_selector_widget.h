#pragma once

#include "calibmar/core/camera_models.h"
#include "ui/widgets/housing_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class HousingSelectorWidget : public QGroupBox {
   public:
    HousingSelectorWidget(QWidget* parent = nullptr);

    std::optional<std::pair<HousingInterfaceType, std::string>> HousingOptions();
    void SetHousingOptions(const std::optional<std::pair<HousingInterfaceType, std::string>>& options);

   private:
    HousingWidget* housing_model_;
    QLabel* housing_parameters_label_;
    QLineEdit* initial_parameters_edit_;
  };
}
