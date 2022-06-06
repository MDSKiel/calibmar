#pragma once

#include "calibmar/core/camera_models.h"
#include "ui/widgets/initial_parameters_widget.h"

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
    void SetCameraParametersLabel(int index);

    std::vector<std::tuple<HousingInterfaceType, std::string, std::string>> housing_models_;
    QComboBox* housing_model_combobox_;
    QLabel* housing_parameters_label_;
    QLineEdit* initial_parameters_edit_;
  };
}
