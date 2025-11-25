
#include "ui/widgets/aruco_board_target_widget.h"

namespace calibmar {

  ArucoBoardTargetOptionsWidget::ArucoBoardTargetOptionsWidget(QWidget* parent) : QWidget(parent) {
    QLabel* aruco_type_label = new QLabel("Aruco Marker Type", this);
    aruco_type_edit_ = new QComboBox(this);
    for (const auto& name_type : calibration_targets::ArucoTypes()) {
      aruco_type_edit_->addItem(QString::fromStdString(name_type.first));
    }

    QLabel* aruco_border_label = new QLabel("Marker Border Bits", this);
    aruco_border_edit_ = new QSpinBox(this);
    aruco_border_edit_->setRange(1, 10);
    aruco_border_edit_->setSingleStep(1);

    QLabel* columns_label = new QLabel(this);
    columns_label->setText("Number of Marker Columns");
    columns_edit_ = new QSpinBox(this);
    columns_edit_->setRange(1, 100);
    columns_edit_->setValue(4);
    columns_edit_->setSingleStep(1);

    QLabel* rows_label = new QLabel(this);
    rows_label->setText("Number of Marker Rows");
    rows_edit_ = new QSpinBox(this);
    rows_edit_->setRange(1, 100);
    rows_edit_->setValue(4);
    rows_edit_->setSingleStep(1);

    QLabel* board_origin_label = new QLabel("Board Origin", this);
    board_origin_edit_ = new QComboBox(this);
    board_origin_edit_->addItem("Top Left");
    board_origin_edit_->addItem("Top Right");
    board_origin_edit_->addItem("Bottom Left");
    board_origin_edit_->addItem("Bottom Right");

    QLabel* board_id_direction_label = new QLabel("Marker IDs Ascending", this);
    board_id_direction_edit_ = new QComboBox(this);
    board_id_direction_edit_->addItem("Horizontal");
    board_id_direction_edit_->addItem("Vertical");

    QLabel* marker_size_label = new QLabel(this);
    marker_size_label->setText("Marker Size");
    marker_size_edit_ = new QDoubleSpinBox(this);
    marker_size_edit_->setDecimals(3);
    marker_size_edit_->setRange(0.001, 1000);
    marker_size_edit_->setSingleStep(0.001);
    marker_size_edit_->setValue(1.0);

    QLabel* spacing_size_label = new QLabel(this);
    spacing_size_label->setText("Marker Spacing");
    spacing_size_edit_ = new QDoubleSpinBox(this);
    spacing_size_edit_->setDecimals(3);
    spacing_size_edit_->setRange(0.001, 1000);
    spacing_size_edit_->setSingleStep(0.001);
    spacing_size_edit_->setValue(1.0);
    QFormLayout* formLayout = new QFormLayout(this);
    formLayout->setWidget(1, QFormLayout::LabelRole, aruco_type_label);
    formLayout->setWidget(1, QFormLayout::FieldRole, aruco_type_edit_);
    formLayout->setWidget(2, QFormLayout::LabelRole, aruco_border_label);
    formLayout->setWidget(2, QFormLayout::FieldRole, aruco_border_edit_);
    formLayout->setWidget(3, QFormLayout::LabelRole, columns_label);
    formLayout->setWidget(3, QFormLayout::FieldRole, columns_edit_);
    formLayout->setWidget(4, QFormLayout::LabelRole, rows_label);
    formLayout->setWidget(4, QFormLayout::FieldRole, rows_edit_);
    formLayout->setWidget(5, QFormLayout::LabelRole, board_origin_label);
    formLayout->setWidget(5, QFormLayout::FieldRole, board_origin_edit_);
    formLayout->setWidget(6, QFormLayout::LabelRole, board_id_direction_label);
    formLayout->setWidget(6, QFormLayout::FieldRole, board_id_direction_edit_);
    formLayout->setWidget(7, QFormLayout::LabelRole, marker_size_label);
    formLayout->setWidget(7, QFormLayout::FieldRole, marker_size_edit_);
    formLayout->setWidget(8, QFormLayout::LabelRole, spacing_size_label);
    formLayout->setWidget(8, QFormLayout::FieldRole, spacing_size_edit_);

    formLayout->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
  }

  void ArucoBoardTargetOptionsWidget::SetArucoBoardTargetOptions(const ArucoBoardFeatureExtractor::Options& options) {
    columns_edit_->setValue(options.marker_cols);
    rows_edit_->setValue(options.marker_rows);
    marker_size_edit_->setValue(options.marker_size);
    spacing_size_edit_->setValue(options.marker_spacing);

    for (size_t i = 0; i < calibration_targets::ArucoTypes().size(); i++) {
      const auto& name_type = calibration_targets::ArucoTypes()[i];
      if (name_type.second == options.aruco_type) {
        aruco_type_edit_->setCurrentIndex(i);
        break;
      }
    }

    aruco_border_edit_->setValue(options.border_bits);

    switch (options.grid_origin) {
      case ArucoGridOrigin::TopLeft:
        board_origin_edit_->setCurrentIndex(0);
        break;
      case ArucoGridOrigin::TopRight:
        board_origin_edit_->setCurrentIndex(1);
        break;
      case ArucoGridOrigin::BottomLeft:
        board_origin_edit_->setCurrentIndex(2);
        break;
      case ArucoGridOrigin::BottomRight:
        board_origin_edit_->setCurrentIndex(3);
        break;
      default:
        throw std::runtime_error("Unkown board origin!");
    }

    switch (options.grid_direction) {
      case ArucoGridDirection::Horizontal:
        board_id_direction_edit_->setCurrentIndex(0);
        break;
      case ArucoGridDirection::Vertical:
        board_id_direction_edit_->setCurrentIndex(1);
        break;
      default:
        throw std::runtime_error("Unkown board direction!");
    }
  }

  ArucoBoardFeatureExtractor::Options ArucoBoardTargetOptionsWidget::ArucoBoardTargetOptions() {
    ArucoBoardFeatureExtractor::Options options;
    options.marker_cols = columns_edit_->value();
    options.marker_rows = rows_edit_->value();
    options.marker_size = marker_size_edit_->value();
    options.marker_spacing = spacing_size_edit_->value();
    options.aruco_type = calibration_targets::ArucoTypeFromName(aruco_type_edit_->currentText().toStdString());

    options.border_bits = aruco_border_edit_->value();

    switch (board_origin_edit_->currentIndex()) {
      case 0:
        options.grid_origin = ArucoGridOrigin::TopLeft;
        break;
      case 1:
        options.grid_origin = ArucoGridOrigin::TopRight;
        break;
      case 2:
        options.grid_origin = ArucoGridOrigin::BottomLeft;
        break;
      case 3:
        options.grid_origin = ArucoGridOrigin::BottomRight;
        break;
    }

    if (board_id_direction_edit_->currentIndex() == 0) {
      options.grid_direction = ArucoGridDirection::Horizontal;
    }
    else {
      options.grid_direction = ArucoGridDirection::Vertical;
    }

    return options;
  }
}