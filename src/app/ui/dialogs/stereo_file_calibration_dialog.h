#pragma once

#include "ui/widgets/common_calibration_options_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class StereoFileCalibrationDialog : public QDialog {
   public:
    struct Options {
      ChessboardFeatureExtractor::Options calibration_target_options;
      CameraModelType camera_model;
      std::optional<std::pair<std::vector<double>, std::vector<double>>> initial_camera_parameters;
      bool estimate_pose_only;
      std::string images_directory1;
      std::string images_directory2;
    };

    StereoFileCalibrationDialog(QWidget* parent = nullptr);

    void SetOptions(Options options);
    Options GetOptions();

   private:
    bool Validate();
    void ImportParameters();

    QLineEdit* directory_edit1_;
    QLineEdit* directory_edit2_;

    CameraModelWidget* camera_model_;
    QCheckBox* use_initial_parameters_checkbox_;
    QCheckBox* only_estimate_pose_checkbox_;
    InitialParametersWidget* initial_parameters_1_;
    InitialParametersWidget* initial_parameters_2_;
    ChessboardTargetOptionsWidget* calibration_target_options_;
  };
}
