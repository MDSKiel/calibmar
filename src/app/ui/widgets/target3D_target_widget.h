#pragma once

#include <QtCore>
#include <QtWidgets>
#include <optional>

#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/sift_extractor.h"

namespace calibmar {

  class Target3DTargetOptionsWidget : public QWidget {
   public:
    typedef std::variant<ArucoSiftFeatureExtractor::Options, SiftFeatureExtractor::Options> Options;

    Target3DTargetOptionsWidget(QWidget* parent = nullptr);

    void SetTarget3DTargetOptions(const Options& options);
    Options Target3DTargetOptions();

    void ForceArucoFor3DTarget(bool force);

   private:
    QDoubleSpinBox* aruco_mask_factor_edit_;
    QComboBox* aruco_type_edit_;
    QCheckBox* enable_aruco_checkbox_;

    void EnableArucoStateChanged(int state);
  };
}