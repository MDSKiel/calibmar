
#include "ui/widgets/calibration_target_widget.h"
#include "calibration_target_widget.h"

namespace calibmar {

  CalibrationTargetOptionsWidget::CalibrationTargetOptionsWidget(QWidget* parent) : QGroupBox(parent) {
    setTitle("Calibration Target");

    QLabel* target_type_label = new QLabel(this);
    target_type_label->setText("Calibration Target Type");
    target_type_combobox_ = new QComboBox(this);
    target_type_combobox_->addItem("Chessboard");
    target_type_combobox_->addItem("3D Target");
    target_type_combobox_->setCurrentIndex(0);

    target3D_target_widget_ = new Target3DTargetOptionsWidget(this);
    target3D_target_widget_->layout()->setContentsMargins(0, 0, 0, 0);
    chessboard_target_widget_ = new ChessboardTargetOptionsWidget(this);
    chessboard_target_widget_->layout()->setContentsMargins(0, 0, 0, 0);

    connect(target_type_combobox_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &CalibrationTargetOptionsWidget::SetCalibrationTargetType);

    // Type selection layout
    QFormLayout* formLayout_target_type = new QFormLayout();
    formLayout_target_type->setWidget(0, QFormLayout::LabelRole, target_type_label);
    formLayout_target_type->setWidget(0, QFormLayout::FieldRole, target_type_combobox_);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addLayout(formLayout_target_type);
    layout->addWidget(chessboard_target_widget_);
    layout->addWidget(target3D_target_widget_);

    target_type_combobox_->setCurrentIndex(0);
    target3D_target_widget_->setVisible(false);
  }

  void CalibrationTargetOptionsWidget::SetCalibrationTargetOptions(const CalibrationTargetOptionsWidget::Options& options) {
    if (std::holds_alternative<ChessboardFeatureExtractor::Options>(options)) {
      target_type_combobox_->setCurrentIndex(0);
      chessboard_target_widget_->SetChessBoardTargetOptions(std::get<ChessboardFeatureExtractor::Options>(options));
    }
    else if (std::holds_alternative<ArucoSiftFeatureExtractor::Options>(options)) {
      target_type_combobox_->setCurrentIndex(1);
      target3D_target_widget_->SetTarget3DTargetOptions(std::get<ArucoSiftFeatureExtractor::Options>(options));
    }
    else {
      target_type_combobox_->setCurrentIndex(1);
      target3D_target_widget_->SetTarget3DTargetOptions(std::get<SiftFeatureExtractor::Options>(options));
    }
  }

  CalibrationTargetOptionsWidget::Options CalibrationTargetOptionsWidget::CalibrationTargetOptions() {
    Options options;
    if (target_type_combobox_->currentIndex() == 0) {
      options = chessboard_target_widget_->ChessboardTargetOptions();
    }
    else {
      options = std::visit([](auto&& arg) -> Options { return arg; }, target3D_target_widget_->Target3DTargetOptions());
    }

    return options;
  }

  void CalibrationTargetOptionsWidget::ForceArucoFor3DTarget(bool force) {
    target3D_target_widget_->ForceArucoFor3DTarget(force);
  }

  void CalibrationTargetOptionsWidget::SetCalibrationTargetType(int index) {
    // depending on index show either of the target widets
    bool chessboard_visible = index == 0;
    // first hide both and the show only one to make sure the parent window never has to show both simultaneously
    target3D_target_widget_->setVisible(false);
    chessboard_target_widget_->setVisible(false);
    target3D_target_widget_->setVisible(!chessboard_visible);
    chessboard_target_widget_->setVisible(chessboard_visible);
  }
}