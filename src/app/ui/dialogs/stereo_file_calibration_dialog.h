#pragma once

#include "ui/widgets/common_calibration_options_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class StereoFileCalibrationDialog : public QDialog {
   public:
    struct Options {
      CalibrationTargetOptionsWidget::Options calibration_target_options;
      CameraModelType camera_model1;
      CameraModelType camera_model2;
      std::optional<std::vector<double>> initial_camera_parameters1;
      std::optional<std::vector<double>> initial_camera_parameters2;
      bool estimate_pose_only = false;
      std::string images_directory1;
      std::string images_directory2;
    };

    StereoFileCalibrationDialog(QWidget* parent = nullptr);

    void SetOptions(Options options);
    Options GetOptions();

   private:
    bool Validate();
    void ImportParameters(int camera);

    QLineEdit* directory_edit1_;
    QLineEdit* directory_edit2_;

    CameraModelWidget* camera_model1_;
    CameraModelWidget* camera_model2_;
    QCheckBox* use_initial_parameters_checkbox1_;
    QCheckBox* use_initial_parameters_checkbox2_;
    QCheckBox* only_estimate_pose_checkbox_;
    InitialParametersWidget* initial_parameters_1_;
    InitialParametersWidget* initial_parameters_2_;
    CalibrationTargetOptionsWidget* calibration_target_options_;
  };
}
