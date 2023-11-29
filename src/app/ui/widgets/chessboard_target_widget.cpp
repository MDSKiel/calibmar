
#include "ui/widgets/chessboard_target_widget.h"
#include "chessboard_target_widget.h"

namespace calibmar {

  ChessboardTargetOptionsWidget::ChessboardTargetOptionsWidget(QWidget* parent) : QWidget(parent) {
    QLabel* chessboard_columns_label = new QLabel(this);
    chessboard_columns_label->setText("Number of Chessboard Columns");
    chess_board_columns_edit_ = new QSpinBox(this);
    chess_board_columns_edit_->setRange(4, 100);
    chess_board_columns_edit_->setSingleStep(1);
    QLabel* chessboard_rows_label = new QLabel(this);
    chessboard_rows_label->setText("Number of Chessboard Rows");
    chess_board_rows_edit_ = new QSpinBox(this);
    chess_board_rows_edit_->setRange(4, 100);
    chess_board_rows_edit_->setSingleStep(1);
    QLabel* square_size_label = new QLabel(this);
    square_size_label->setText("Square Size");
    square_size_edit_ = new QDoubleSpinBox(this);
    square_size_edit_->setDecimals(3);
    square_size_edit_->setRange(0.001, 1000);
    square_size_edit_->setSingleStep(0.001);
    square_size_edit_->setValue(1.0);
    QFormLayout* formLayout_chessboard = new QFormLayout(this);
    formLayout_chessboard->setWidget(0, QFormLayout::LabelRole, chessboard_columns_label);
    formLayout_chessboard->setWidget(0, QFormLayout::FieldRole, chess_board_columns_edit_);
    formLayout_chessboard->setWidget(1, QFormLayout::LabelRole, chessboard_rows_label);
    formLayout_chessboard->setWidget(1, QFormLayout::FieldRole, chess_board_rows_edit_);
    formLayout_chessboard->setWidget(2, QFormLayout::LabelRole, square_size_label);
    formLayout_chessboard->setWidget(2, QFormLayout::FieldRole, square_size_edit_);
    formLayout_chessboard->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
  }

  void ChessboardTargetOptionsWidget::SetChessBoardTargetOptions(const ChessboardFeatureExtractor::Options& options) {
    chess_board_columns_edit_->setValue(options.chessboard_columns);
    chess_board_rows_edit_->setValue(options.chessboard_rows);
    square_size_edit_->setValue(options.square_size);
  }

  ChessboardFeatureExtractor::Options ChessboardTargetOptionsWidget::ChessboardTargetOptions() {
    ChessboardFeatureExtractor::Options options;
    options.chessboard_columns = chess_board_columns_edit_->value();
    options.chessboard_rows = chess_board_rows_edit_->value();
    options.square_size = square_size_edit_->value();

    return options;
  }
}