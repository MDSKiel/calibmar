#pragma once

#include "calibmar/calibrators/calibrator.h"
#include "ui/widgets/common_calibration_options_widget.h"
#include "ui/widgets/image_widget.h"

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class StreamCalibrationDialog : public QDialog {
   public:
    enum class AcquisitionMode { OnButton, OnTimedDetection, OnSuggestedPose };

    struct Options {
      int chessboard_rows = 4;
      int chessboard_columns = 4;
      double square_size = 1.0;
      CameraModelType camera_model = CameraModelType::SimplePinholeCameraModel;
      std::optional<std::pair<HousingInterfaceType, std::vector<double>>> housing_calibration = {};
      std::optional<std::vector<double>> initial_camera_parameters = {};
      std::optional<std::string> save_images_directory = {};
      int device_index = 0;
      AcquisitionMode acquisition_mode = AcquisitionMode::OnTimedDetection;
    };

    StreamCalibrationDialog(QWidget* parent = nullptr);

    Options GetOptions();

    virtual void done(int r) override;

   private:
    bool Validate();
    void ImportParameters();
    void StartLiveStream(int index);
    void CloseLiveStream();

    std::vector<std::pair<AcquisitionMode, std::string>> acquisition_modes_;
    std::unique_ptr<std::thread> worker_thread_;
    std::atomic<bool> cancel_;
    std::atomic<bool> valid_device_;

    std::optional<std::string> save_images_directory_;

    QComboBox* mode_combobox_;
    QSpinBox* device_index_;
    CommonCalibrationOptionsWidget* calibration_options_widget_;
    ImageWidget* image_widget_;
    QLineEdit* directory_edit_;
    QCheckBox* save_images_checkbox_;
  };
}
