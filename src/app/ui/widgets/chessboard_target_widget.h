#pragma once

#include <QtCore>
#include <QtWidgets>
#include "calibmar/extractors/chessboard_extractor.h"

namespace calibmar {

  class ChessboardTargetOptionsWidget : public QWidget {
   public:
    ChessboardTargetOptionsWidget(QWidget* parent = nullptr);

    void SetChessBoardTargetOptions(const ChessboardFeatureExtractor::Options& options);
    ChessboardFeatureExtractor::Options ChessboardTargetOptions();
   private:
    QSpinBox* chess_board_rows_edit_;
    QSpinBox* chess_board_columns_edit_;
    QDoubleSpinBox* square_size_edit_;
  };
}