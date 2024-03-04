#include "stereo_file_calibration_dialog.h"

#include "ui/utils/parse_params.h"
#include <calibmar/core/report.h>
#include <colmap/util/misc.h>
#include <filesystem>

namespace {
  std::optional<std::string> ConvertInitialParameters(const std::optional<std::vector<double>>& parameters) {
    return parameters.has_value() ? colmap::VectorToCSV(*parameters) : std::optional<std::string>();
  }

  std::optional<std::vector<double>> ParseInitialParameters(const std::optional<std::string>& parameters_string) {
    std::vector<double> params;
    if (!parameters_string.has_value() || !calibmar::TryParseParams(params, parameters_string.value())) {
      return {};
    }

    return params;
  }

  bool ValidateParameters(const std::optional<std::string>& parameters_string, calibmar::CameraModelType camera_model,
                          std::string& error_message) {
    if (parameters_string.has_value()) {
      std::vector<double> params;
      if (!calibmar::TryParseParams(params, parameters_string.value())) {
        error_message = "Invalid camera parameter format.";
        return false;
      }
      else if (params.size() != calibmar::CameraModel::CameraModels().at(camera_model).num_params) {
        error_message = "Camera parameters dont match camera model.";
        return false;
      }
    }

    return true;
  }
}

namespace calibmar {

  StereoFileCalibrationDialog::StereoFileCalibrationDialog(QWidget* parent) : QDialog(parent) {
    // directory groupbox
    QGroupBox* directory_groupbox = new QGroupBox(this);
    directory_groupbox->setTitle("Image files directory");
    directory_edit1_ = new QLineEdit(directory_groupbox);
    directory_edit2_ = new QLineEdit(directory_groupbox);
    QPushButton* select_directory_button1 = new QPushButton(directory_groupbox);
    QPushButton* select_directory_button2 = new QPushButton(directory_groupbox);
    select_directory_button1->setText("Browse");
    select_directory_button2->setText("Browse");
    connect(select_directory_button1, &QPushButton::released, this, [this]() {
      this->directory_edit1_->setText(QFileDialog::getExistingDirectory(this, "Select calibration target image directory"));
    });
    connect(select_directory_button2, &QPushButton::released, this, [this]() {
      this->directory_edit2_->setText(QFileDialog::getExistingDirectory(this, "Select calibration target image directory"));
    });
    QGridLayout* layout_directory = new QGridLayout(directory_groupbox);
    layout_directory->addWidget(directory_edit1_, 0, 0);
    layout_directory->addWidget(select_directory_button1, 0, 1);
    layout_directory->addWidget(directory_edit2_, 1, 0);
    layout_directory->addWidget(select_directory_button2, 1, 1);

    // Camera Model & Intrinsics
    QGroupBox* camera_model_groupbox = new QGroupBox("Camera Model");
    QGridLayout* camera_model_layout = new QGridLayout(camera_model_groupbox);
    camera_model_ = new CameraModelWidget(camera_model_groupbox);
    use_initial_parameters_checkbox_ = new QCheckBox(camera_model_groupbox);
    initial_parameters_1_ = new InitialParametersWidget(camera_model_groupbox, false);
    initial_parameters_2_ = new InitialParametersWidget(camera_model_groupbox, false);
    connect(use_initial_parameters_checkbox_, &QCheckBox::stateChanged, this, [this](int state) {
      initial_parameters_1_->SetChecked(use_initial_parameters_checkbox_->isChecked());
      initial_parameters_2_->SetChecked(use_initial_parameters_checkbox_->isChecked());
    });

    only_estimate_pose_checkbox_ = new QCheckBox("Only estimate relative pose", camera_model_groupbox);

    camera_model_layout->addWidget(camera_model_, 0, 0, 1, 2);
    camera_model_layout->addWidget(use_initial_parameters_checkbox_, 1, 0);
    camera_model_layout->addWidget(initial_parameters_1_, 1, 1);
    camera_model_layout->addWidget(initial_parameters_2_, 2, 1);
    camera_model_layout->addWidget(only_estimate_pose_checkbox_, 3, 0, 1, 2);

    // chessboard target
    QGroupBox* chessboard_groupbox = new QGroupBox("Chessboard target");
    QVBoxLayout* chessboard_layout = new QVBoxLayout(chessboard_groupbox);
    calibration_target_options_ = new ChessboardTargetOptionsWidget(chessboard_groupbox);
    chessboard_layout->addWidget(calibration_target_options_);
    chessboard_layout->setContentsMargins(0, 0, 0, 0);

    // import button
    QHBoxLayout* horizontalLayout_run = new QHBoxLayout();
    QPushButton* import_button = new QPushButton(this);
    import_button->setText("Import...");
    connect(import_button, &QPushButton::released, this, [this]() { ImportParameters(); });
    horizontalLayout_run->addWidget(import_button, 0, Qt::AlignLeft | Qt::AlignTop);

    // run button
    QPushButton* run_button = new QPushButton(this);
    run_button->setText("Run");
    run_button->setDefault(true);
    connect(run_button, &QPushButton::released, this, [this]() {
      if (Validate()) {
        this->accept();
      }
    });

    horizontalLayout_run->addWidget(run_button, 0, Qt::AlignRight | Qt::AlignTop);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(directory_groupbox);
    layout->addWidget(camera_model_groupbox);
    layout->addWidget(chessboard_groupbox);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Stereo Calibrate from Files");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  void StereoFileCalibrationDialog::SetOptions(Options options) {
    directory_edit1_->setText(QString::fromStdString(options.images_directory1));
    directory_edit2_->setText(QString::fromStdString(options.images_directory2));

    only_estimate_pose_checkbox_->setChecked(options.estimate_pose_only);

    camera_model_->SetCameraModel(options.camera_model);
    if (options.initial_camera_parameters.has_value()) {
      initial_parameters_1_->SetInitialParameters(ConvertInitialParameters(options.initial_camera_parameters->first));
      initial_parameters_2_->SetInitialParameters(ConvertInitialParameters(options.initial_camera_parameters->second));
      use_initial_parameters_checkbox_->setChecked(true);
    }
    else {
      initial_parameters_1_->SetInitialParameters({});
      initial_parameters_2_->SetInitialParameters({});
      use_initial_parameters_checkbox_->setChecked(false);
    }

    calibration_target_options_->SetChessBoardTargetOptions(options.calibration_target_options);
  }

  StereoFileCalibrationDialog::Options StereoFileCalibrationDialog::GetOptions() {
    Options options;
    options.camera_model = camera_model_->CameraModel();

    if (use_initial_parameters_checkbox_->isChecked()) {
      std::optional<std::vector<double>> params1 = ParseInitialParameters(initial_parameters_1_->InitialParameters());
      std::optional<std::vector<double>> params2 = ParseInitialParameters(initial_parameters_2_->InitialParameters());
      options.initial_camera_parameters = {*params1, *params2};
    }
    else {
      options.initial_camera_parameters = {};
    }

    options.estimate_pose_only = only_estimate_pose_checkbox_->isChecked();

    options.images_directory1 = directory_edit1_->text().toStdString();
    options.images_directory2 = directory_edit2_->text().toStdString();
    options.calibration_target_options = calibration_target_options_->ChessboardTargetOptions();
    return options;
  }

  bool StereoFileCalibrationDialog::Validate() {
    if (!std::filesystem::is_directory(directory_edit1_->text().toStdString())) {
      QMessageBox::information(this, "Validation Error", "First image directory does not exist.");
      return false;
    }

    if (!std::filesystem::is_directory(directory_edit2_->text().toStdString())) {
      QMessageBox::information(this, "Validation Error", "Second image directory does not exist.");
      return false;
    }

    std::string message;
    CameraModelType camera_model = camera_model_->CameraModel();
    if (!ValidateParameters(initial_parameters_1_->InitialParameters(), camera_model, message) ||
        !ValidateParameters(initial_parameters_2_->InitialParameters(), camera_model, message)) {
      QMessageBox::information(this, "Validation Error", QString::fromStdString(message));
      return false;
    }

    if (only_estimate_pose_checkbox_->isChecked() && !use_initial_parameters_checkbox_->isChecked()) {
      QMessageBox::information(this, "Validation Error", "To estimate the relative pose only, intrinsics must be provided!");
      return false;
    }

    return true;
  }

  void StereoFileCalibrationDialog::ImportParameters() {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();
    if (path.empty()) {
      return;
    }

    ImportedParameters parameters = ImportedParameters::ImportFromYaml(path);
    Options options;

    options.calibration_target_options.chessboard_columns = parameters.chessboard_columns;
    options.calibration_target_options.chessboard_rows = parameters.chessboard_rows;
    options.calibration_target_options.square_size = parameters.square_size;

    options.images_directory1 = parameters.directory;
    options.images_directory2 = parameters.directory;

    options.camera_model = parameters.camera_model;
    options.initial_camera_parameters = {parameters.camera_parameters, parameters.camera_parameters};

    SetOptions(options);
  }
}