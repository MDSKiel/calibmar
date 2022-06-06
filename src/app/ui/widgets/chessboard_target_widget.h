#pragma once

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class ChessboardTargetOptionsWidget : public QGroupBox {
   public:
    ChessboardTargetOptionsWidget(QWidget* parent = nullptr);

    void SetChessBoardTargetOptions(int columns, int rows, double square_size);

    int ChessboardColumns();
    int ChessboardRows();
    double SquareSize();

   private:
    QSpinBox* chess_board_rows_edit_;
    QSpinBox* chess_board_columns_edit_;
    QDoubleSpinBox* square_size_edit_;
  };
}