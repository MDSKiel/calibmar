#pragma once

#include "calibmar/extractors/aruco_board_extractor.h"
#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class ArucoBoardTargetOptionsWidget : public QWidget {
   public:
    ArucoBoardTargetOptionsWidget(QWidget* parent = nullptr);

    void SetArucoBoardTargetOptions(const ArucoBoardFeatureExtractor::Options& options);
    ArucoBoardFeatureExtractor::Options ArucoBoardTargetOptions();

   private:
    QComboBox* aruco_type_edit_;
    QSpinBox* aruco_border_edit_;
    QSpinBox* rows_edit_;
    QSpinBox* columns_edit_;
    QComboBox* board_origin_edit_;
    QComboBox* board_id_direction_edit_;
    QDoubleSpinBox* marker_size_edit_;
    QDoubleSpinBox* spacing_size_edit_;
  };
}