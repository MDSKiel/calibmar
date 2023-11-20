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
    chessboard_target_options_ = new ChessboardTargetOptionsWidget(this);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(camera_model_selector_);
    layout->addWidget(housing_type_selector_);
    layout->addWidget(chessboard_target_options_);
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
    chessboard_target_options_->SetChessBoardTargetOptions(parameters.chessboard_columns, parameters.chessboard_rows,
                                                           parameters.square_size);
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

  int CommonCalibrationOptionsWidget::ChessboardColumns() {
    return chessboard_target_options_->ChessboardColumns();
  }

  int CommonCalibrationOptionsWidget::ChessboardRows() {
    return chessboard_target_options_->ChessboardRows();
  }

  double CommonCalibrationOptionsWidget::SquareSize() {
    return chessboard_target_options_->SquareSize();
  }
}