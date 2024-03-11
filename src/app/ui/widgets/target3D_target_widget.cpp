
#include "ui/widgets/target3D_target_widget.h"
#include "calibmar/core/calibration_targets.h"
#include "target3D_target_widget.h"

#include <algorithm>

namespace calibmar {
  Target3DTargetOptionsWidget::Target3DTargetOptionsWidget(QWidget* parent) : QWidget(parent) {
    QLabel* enable_aruco_label = new QLabel("Enable Aruco Detection", this);
    enable_aruco_checkbox_ = new QCheckBox(this);

    connect(enable_aruco_checkbox_, &QCheckBox::stateChanged, this, &Target3DTargetOptionsWidget::EnableArucoStateChanged);

    QLabel* aruco_type_label = new QLabel("Aruco Marker Type", this);
    aruco_type_edit_ = new QComboBox(this);

    for (const auto& name_type : calibration_targets::ArucoTypes()) {
      aruco_type_edit_->addItem(QString::fromStdString(name_type.first));
    }

    QLabel* aruco_mask_factor_label = new QLabel("Aruco Mask Factor", this);
    aruco_mask_factor_edit_ = new QDoubleSpinBox(this);
    aruco_mask_factor_edit_->setDecimals(3);
    aruco_mask_factor_edit_->setRange(1, 100);
    aruco_mask_factor_edit_->setSingleStep(0.001);

    QFormLayout* layout = new QFormLayout(this);
    layout->setWidget(0, QFormLayout::LabelRole, enable_aruco_label);
    layout->setWidget(0, QFormLayout::FieldRole, enable_aruco_checkbox_);
    layout->setWidget(1, QFormLayout::LabelRole, aruco_type_label);
    layout->setWidget(1, QFormLayout::FieldRole, aruco_type_edit_);
    layout->setWidget(2, QFormLayout::LabelRole, aruco_mask_factor_label);
    layout->setWidget(2, QFormLayout::FieldRole, aruco_mask_factor_edit_);
    layout->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);

    SetTarget3DTargetOptions(Options());
  }

  void Target3DTargetOptionsWidget::SetTarget3DTargetOptions(const Options& options) {
    bool use_aruco = std::holds_alternative<ArucoSiftFeatureExtractor::Options>(options);
    enable_aruco_checkbox_->setChecked(use_aruco);

    if (use_aruco) {
      ArucoSiftFeatureExtractor::Options aruco_options = std::get<ArucoSiftFeatureExtractor::Options>(options);

      for (size_t i = 0; i < calibration_targets::ArucoTypes().size(); i++) {
        const auto& name_type = calibration_targets::ArucoTypes()[i];
        if (name_type.second == aruco_options.aruco_type) {
          aruco_type_edit_->setCurrentIndex(i);
          break;
        }
      }
      aruco_mask_factor_edit_->setValue(aruco_options.masking_scale_factor);
    }

    EnableArucoStateChanged(use_aruco ? Qt::Checked : Qt::Unchecked);
  }

  Target3DTargetOptionsWidget::Options Target3DTargetOptionsWidget::Target3DTargetOptions() {
    if (enable_aruco_checkbox_->isChecked()) {
      ArucoSiftFeatureExtractor::Options options;

      options.aruco_type = calibration_targets::ArucoTypeFromName(aruco_type_edit_->currentText().toStdString());

      options.masking_scale_factor = aruco_mask_factor_edit_->value();
      return options;
    }
    else {
      return SiftFeatureExtractor::Options{};
    }
  }

  void Target3DTargetOptionsWidget::ForceArucoFor3DTarget(bool force) {
    if (force) {
      // EnableArucoStateChanged(Qt::Checked);
      enable_aruco_checkbox_->setChecked(true);
    }

    enable_aruco_checkbox_->setEnabled(!force);
  }

  void Target3DTargetOptionsWidget::EnableArucoStateChanged(int state) {
    bool active = state != Qt::Unchecked;
    aruco_mask_factor_edit_->setEnabled(active);
    aruco_type_edit_->setEnabled(active);
  }
}