#pragma once

#include "calibmar/core/camera_models.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class HousingWidget : public QWidget {
   public:
    HousingWidget(QWidget* parent = nullptr);
    HousingWidget(const std::function<void()> model_changed_callback, QWidget* parent = nullptr);

    std::optional<HousingInterfaceType> HousingType();
    void SetHousingType(const std::optional<HousingInterfaceType>& options);

   private:
    void SetHousingParametersLabel(int index);

    std::vector<std::tuple<HousingInterfaceType, std::string, std::string>> housing_models_;
    QComboBox* housing_model_combobox_;
    QLabel* housing_parameters_label_;

    std::function<void()> model_changed_callback_;
  };
}
