
#include "ui/widgets/calibration_target_widget.h"
#include "calibration_target_widget.h"

namespace calibmar {

  CalibrationTargetOptionsWidget::CalibrationTargetOptionsWidget(QWidget* parent) : QGroupBox(parent) {
    setTitle("Calibration Target");

    QLabel* target_type_label = new QLabel(this);
    target_type_label->setText("Calibration Target Type");
    target_type_combobox_ = new QComboBox(this);
    target_type_combobox_->addItem("Chessboard");
    target_type_combobox_->addItem("Aruco Grid Board");
    target_type_combobox_->addItem("3D Target");
    target_type_combobox_->setCurrentIndex(0);

    target3D_target_widget_ = new Target3DTargetOptionsWidget(this);
    target3D_target_widget_->layout()->setContentsMargins(0, 0, 0, 0);
    chessboard_target_widget_ = new ChessboardTargetOptionsWidget(this);
    chessboard_target_widget_->layout()->setContentsMargins(0, 0, 0, 0);
    aruco_board_target_widget_ = new ArucoBoardTargetOptionsWidget(this);
    aruco_board_target_widget_->layout()->setContentsMargins(0, 0, 0, 0);

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
    layout->addWidget(aruco_board_target_widget_);
    layout->addWidget(target3D_target_widget_);

    target_type_combobox_->setCurrentIndex(0);
    SetCalibrationTargetType(0);
  }

  void CalibrationTargetOptionsWidget::SetCalibrationTargetOptions(const CalibrationTargetOptionsWidget::Options& options) {
    if (std::holds_alternative<ChessboardFeatureExtractor::Options>(options)) {
      target_type_combobox_->setCurrentIndex(0);
      chessboard_target_widget_->SetChessBoardTargetOptions(std::get<ChessboardFeatureExtractor::Options>(options));
    }
    else if (std::holds_alternative<ArucoBoardFeatureExtractor::Options>(options)) {
      target_type_combobox_->setCurrentIndex(1);
      aruco_board_target_widget_->SetArucoBoardTargetOptions(std::get<ArucoBoardFeatureExtractor::Options>(options));
    }
    else if (std::holds_alternative<ArucoSiftFeatureExtractor::Options>(options)) {
      target_type_combobox_->setCurrentIndex(2);
      target3D_target_widget_->SetTarget3DTargetOptions(std::get<ArucoSiftFeatureExtractor::Options>(options));
    }
    else {
      target_type_combobox_->setCurrentIndex(2);
      target3D_target_widget_->SetTarget3DTargetOptions(std::get<SiftFeatureExtractor::Options>(options));
    }
  }

  CalibrationTargetOptionsWidget::Options CalibrationTargetOptionsWidget::CalibrationTargetOptions() {
    Options options;
    if (target_type_combobox_->currentIndex() == 0) {
      options = chessboard_target_widget_->ChessboardTargetOptions();
    }
    else if (target_type_combobox_->currentIndex() == 1) {
      options = aruco_board_target_widget_->ArucoBoardTargetOptions();
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
    // depending on index show correct target widet
    // NOTE: order in vector here must match index in combobox
    std::vector<QWidget*> widgets{chessboard_target_widget_, aruco_board_target_widget_, target3D_target_widget_};
    for (size_t i = 0; i < widgets.size(); i++) {
      // first hide all to make sure the parent window never has to show multiple simultaneously
      widgets[i]->setVisible(false);
    }

    if (index < widgets.size()) {
      widgets[index]->setVisible(true);
    }
  }
}