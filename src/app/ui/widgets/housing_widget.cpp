#include "ui/widgets/housing_widget.h"
#include "housing_widget.h"

namespace {
  void InitializeHousingModels(
      std::vector<std::tuple<calibmar::HousingInterfaceType, std::string, std::string>>& housing_models) {
    for (auto& [type, housing] : calibmar::HousingInterface::HousingInterfaces()) {
      housing_models.push_back({type, housing.friendly_name, housing.params_info});
    }
  }
}

namespace calibmar {

  HousingWidget::HousingWidget(QWidget* parent) : HousingWidget(nullptr, parent) {}

  HousingWidget::HousingWidget(const std::function<void()> model_changed_callback, QWidget* parent) : QWidget(parent) {
    InitializeHousingModels(housing_models_);

    housing_parameters_label_ = new QLabel(this);
    housing_model_combobox_ = new QComboBox(this);
    housing_model_combobox_->addItem("None");
    for (auto const& tuple : housing_models_) {
      housing_model_combobox_->addItem(QString::fromStdString(std::get<1>(tuple)));
    }
    connect(housing_model_combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &HousingWidget::SetHousingParametersLabel);
    housing_model_combobox_->setCurrentIndex(0);
    SetHousingParametersLabel(0);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(housing_model_combobox_);
    layout->addWidget(housing_parameters_label_);

    // assign last to not call it during construction
    model_changed_callback_ = model_changed_callback;
  }

  std::optional<HousingInterfaceType> HousingWidget::HousingType() {
    int idx = housing_model_combobox_->currentIndex();
    if (idx == 0) {
      // index 0 is "None"
      return {};
    }
    else {
      return std::get<0>(housing_models_[idx - 1]);
    }
  }

  void HousingWidget::SetHousingType(const std::optional<HousingInterfaceType>& options) {
    if (options.has_value()) {
      int index = 0;
      for (size_t i = 0; i < housing_models_.size(); i++) {
        if (std::get<0>(housing_models_[i]) == options.value()) {
          index = i;
          break;
        }
      }
      // accout for NONE offset
      housing_model_combobox_->setCurrentIndex(index + 1);
    }
    else {
      // index 0 is "None"
      housing_model_combobox_->setCurrentIndex(0);
    }
  }

  void HousingWidget::SetHousingParametersLabel(int index) {
    if (index == 0) {
      // index 0 is "None"
      housing_parameters_label_->setVisible(false);
    }
    else {
      housing_parameters_label_->setVisible(true);
      housing_parameters_label_->setText(QString::fromStdString("Parameters: " + std::get<2>(housing_models_[index - 1])));
    }

    if (model_changed_callback_) {
      model_changed_callback_();
    }
  }
}