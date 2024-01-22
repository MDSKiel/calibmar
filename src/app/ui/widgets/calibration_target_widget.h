#pragma once

#include <QtCore>
#include <QtWidgets>
#include <optional>

#include "chessboard_target_widget.h"
#include "target3D_target_widget.h"

namespace calibmar {

  class CalibrationTargetOptionsWidget : public QGroupBox {
   public:
    typedef std::variant<ChessboardFeatureExtractor::Options, ArucoSiftFeatureExtractor::Options, SiftFeatureExtractor::Options> Options;

    CalibrationTargetOptionsWidget(QWidget* parent = nullptr);

    void SetCalibrationTargetOptions(const Options& options);
    Options CalibrationTargetOptions();

    void ForceArucoFor3DTarget(bool force);

   private:
    void SetCalibrationTargetType(int index);

    QComboBox* target_type_combobox_;
    Target3DTargetOptionsWidget* target3D_target_widget_;
    ChessboardTargetOptionsWidget* chessboard_target_widget_;
  };
}