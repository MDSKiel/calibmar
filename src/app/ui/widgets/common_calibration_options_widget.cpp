#include "common_calibration_options_widget.h"
#include <colmap/util/misc.h>

namespace {
  bool TryParseParams(std::vector<double>& params, const std::string& params_string) {
    std::vector<double> parsed = colmap::CSVToVector<double>(params_string);
    if (parsed.size() == 0) {
      return false;
    }
    for (auto d : parsed) {
      params.push_back(d);
    }
    return true;
  }
}

namespace calibmar {
  CommonCalibrationOptionsWidget::CommonCalibrationOptionsWidget(QWidget* parent) : QWidget(parent) {
    // camera model
    camera_model_selector_ = new CameraModelSelectorWidget(this);

    // housing type
    housing_type_selector_ = new HousingSelectorWidget(this);

    // calibration target groupbox
    calibration_target_options_ = new CalibrationTargetOptionsWidget(this);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(camera_model_selector_);
    layout->addWidget(housing_type_selector_);
    layout->addWidget(calibration_target_options_);
    layout->setSizeConstraint(QLayout::SetMinimumSize);

    layout->setContentsMargins(0, 0, 0, 0);
  }

  bool CommonCalibrationOptionsWidget::Validate() {
    std::optional<std::pair<HousingInterfaceType, std::string>> housing = housing_type_selector_->HousingOptions();
    if (housing.has_value()) {
      std::vector<double> params;
      if (!TryParseParams(params, housing.value().second)) {
        QMessageBox::information(this, "Validation Error", "Invalid housing parameter format.");
        return false;
      }
      else if (params.size() != HousingInterface::HousingInterfaces().at(housing.value().first).num_params) {
        QMessageBox::information(this, "Validation Error", "Housing parameters dont match housing type.");
        return false;
      }
      else {
        // validated, take parameters
        housing_calibration_ = {housing.value().first, params};
      }
    }
    else {
      housing_calibration_ = {};
    }
    std::optional<std::string> initial_parameters = camera_model_selector_->InitialCameraParameters();
    CameraModelType camera_model = camera_model_selector_->CameraModel();
    if (initial_parameters.has_value()) {
      std::vector<double> params;
      if (!TryParseParams(params, initial_parameters.value())) {
        QMessageBox::information(this, "Validation Error", "Invalid camera parameter format.");
        return false;
      }
      else if (params.size() != CameraModel::CameraModels().at(camera_model).num_params) {
        QMessageBox::information(this, "Validation Error", "Camera parameters dont match camera model.");
        return false;
      }
      else {
        // validated, take parameters
        initial_camera_parameters_ = params;
      }
    }
    else {
      initial_camera_parameters_ = {};
    }

    if (housing_calibration_.has_value() && !initial_camera_parameters_.has_value()) {
      QMessageBox::information(this, "Validation Error", "Housing calibration requires valid camera parameters.");
      return false;
    }

    return true;
  }

  void CommonCalibrationOptionsWidget::SetImportedParameters(const ImportedParameters& parameters) {
    camera_model_selector_->SetCameraModel(parameters.camera_model);
    std::optional<std::string> params;
    if (parameters.camera_parameters.size() > 0) {
      params = colmap::VectorToCSV<double>(parameters.camera_parameters);
    }
    camera_model_selector_->SetInitialCameraParameters(params);
    std::optional<std::pair<HousingInterfaceType, std::string>> housing_options;
    if (parameters.housing_model.has_value()) {
      housing_options = std::pair<HousingInterfaceType, std::string>(parameters.housing_model.value(),
                                                                     colmap::VectorToCSV<double>(parameters.housing_parameters));
    }
    housing_type_selector_->SetHousingOptions(housing_options);

    switch (parameters.calibration_target) {
      case CalibrationTargetType::Chessboard: {
        ChessboardFeatureExtractor::Options chessboard_options;
        chessboard_options.chessboard_columns = parameters.chessboard_columns;
        chessboard_options.chessboard_rows = parameters.chessboard_rows;
        chessboard_options.square_size = parameters.square_size;
        calibration_target_options_->SetCalibrationTargetOptions(chessboard_options);
        break;
      }
      case CalibrationTargetType::Target3DAruco: {
        ArucoSiftFeatureExtractor::Options aruco_options;
        aruco_options.aruco_type = parameters.aruco_type;
        aruco_options.masking_scale_factor = parameters.aruco_scale_factor;
        calibration_target_options_->SetCalibrationTargetOptions(aruco_options);
        break;
      }
      case CalibrationTargetType::Target3D: {
        calibration_target_options_->SetCalibrationTargetOptions(SiftFeatureExtractor::Options());
        break;
      }
      default:
        break;
    }
  }

  CameraModelType CommonCalibrationOptionsWidget::CameraModel() {
    return camera_model_selector_->CameraModel();
  }

  std::optional<std::vector<double>> CommonCalibrationOptionsWidget::InitialCameraParameters() {
    return initial_camera_parameters_;
  }

  std::optional<std::pair<HousingInterfaceType, std::vector<double>>> CommonCalibrationOptionsWidget::HousingOptions() {
    return housing_calibration_;
  }

  CalibrationTargetOptionsWidget::Options CommonCalibrationOptionsWidget::CalibrationTargetOptions() {
    return calibration_target_options_->CalibrationTargetOptions();
  }

  void CommonCalibrationOptionsWidget::ForceArucoFor3DTarget(bool force) {
    calibration_target_options_->ForceArucoFor3DTarget(force);
  }
}