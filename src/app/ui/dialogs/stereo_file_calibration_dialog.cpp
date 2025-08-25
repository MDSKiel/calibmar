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
    // Camera Model 1
    QGroupBox* camera_model1_groupbox = new QGroupBox("First Camera");
    QVBoxLayout* camera_model1_layout = new QVBoxLayout(camera_model1_groupbox);

    // Directory 1
    directory_edit1_ = new QLineEdit(camera_model1_groupbox);
    directory_edit1_->setPlaceholderText("Image files directory");
    QPushButton* select_directory_button1 = new QPushButton(camera_model1_groupbox);
    select_directory_button1->setText("Browse");
    connect(select_directory_button1, &QPushButton::released, this, [this]() {
      this->directory_edit1_->setText(QFileDialog::getExistingDirectory(this, "Select calibration target image directory"));
    });
    QHBoxLayout* horizontal_layout_directory1 = new QHBoxLayout();
    horizontal_layout_directory1->addWidget(directory_edit1_);
    horizontal_layout_directory1->addWidget(select_directory_button1);
    // Camera Model 1
    camera_model1_ = new CameraModelWidget(camera_model1_groupbox);
    use_initial_parameters_checkbox1_ = new QCheckBox(camera_model1_groupbox);
    initial_parameters_1_ = new InitialParametersWidget(camera_model1_groupbox, false);
    connect(use_initial_parameters_checkbox1_, &QCheckBox::stateChanged, this,
            [this](int state) { initial_parameters_1_->SetChecked(use_initial_parameters_checkbox1_->isChecked()); });
    QHBoxLayout* horizontal_layout_camera1 = new QHBoxLayout();
    horizontal_layout_camera1->addWidget(use_initial_parameters_checkbox1_);
    horizontal_layout_camera1->addWidget(initial_parameters_1_);
    // import button 1
    QPushButton* import_button1 = new QPushButton(this);
    import_button1->setText("Import...");
    connect(import_button1, &QPushButton::released, this, [this]() { ImportParameters(0); });

    camera_model1_layout->addLayout(horizontal_layout_directory1);
    camera_model1_layout->addWidget(camera_model1_);
    camera_model1_layout->addLayout(horizontal_layout_camera1);
    camera_model1_layout->addWidget(import_button1, 0, Qt::AlignLeft);
    // Camera Model 2
    QGroupBox* camera_model2_groupbox = new QGroupBox("Second Camera");
    QVBoxLayout* camera_model2_layout = new QVBoxLayout(camera_model2_groupbox);

    // Directory 2
    directory_edit2_ = new QLineEdit(camera_model2_groupbox);
    directory_edit2_->setPlaceholderText("Image files directory");
    QPushButton* select_directory_button2 = new QPushButton(camera_model2_groupbox);
    select_directory_button2->setText("Browse");
    connect(select_directory_button2, &QPushButton::released, this, [this]() {
      this->directory_edit2_->setText(QFileDialog::getExistingDirectory(this, "Select calibration target image directory"));
    });
    QHBoxLayout* horizontal_layout_directory2 = new QHBoxLayout();
    horizontal_layout_directory2->addWidget(directory_edit2_);
    horizontal_layout_directory2->addWidget(select_directory_button2);
    // Camera Model 2
    camera_model2_ = new CameraModelWidget(camera_model2_groupbox);
    use_initial_parameters_checkbox2_ = new QCheckBox(camera_model2_groupbox);
    initial_parameters_2_ = new InitialParametersWidget(camera_model2_groupbox, false);
    connect(use_initial_parameters_checkbox2_, &QCheckBox::stateChanged, this,
            [this](int state) { initial_parameters_2_->SetChecked(use_initial_parameters_checkbox2_->isChecked()); });
    QHBoxLayout* horizontal_layout_camera2 = new QHBoxLayout();
    horizontal_layout_camera2->addWidget(use_initial_parameters_checkbox2_);
    horizontal_layout_camera2->addWidget(initial_parameters_2_);
    // import button 2
    QPushButton* import_button2 = new QPushButton(this);
    import_button2->setText("Import...");
    connect(import_button2, &QPushButton::released, this, [this]() { ImportParameters(1); });

    camera_model2_layout->addLayout(horizontal_layout_directory2);
    camera_model2_layout->addWidget(camera_model2_);
    camera_model2_layout->addLayout(horizontal_layout_camera2);
    camera_model2_layout->addWidget(import_button2, 0, Qt::AlignLeft);

    // Relative Pose Checkbox
    QGroupBox* stereo_calibration_groupbox = new QGroupBox("Stereo calibration parameters");
    only_estimate_pose_checkbox_ = new QCheckBox("Only estimate relative pose", stereo_calibration_groupbox);
    QHBoxLayout* stereo_parameters_layout = new QHBoxLayout(stereo_calibration_groupbox);
    stereo_parameters_layout->addWidget(only_estimate_pose_checkbox_);

    // chessboard target
    QGroupBox* chessboard_groupbox = new QGroupBox("Chessboard target");
    QVBoxLayout* chessboard_layout = new QVBoxLayout(chessboard_groupbox);
    calibration_target_options_ = new ChessboardTargetOptionsWidget(chessboard_groupbox);
    chessboard_layout->addWidget(calibration_target_options_);
    chessboard_layout->setContentsMargins(0, 0, 0, 0);

    // run button
    QHBoxLayout* horizontalLayout_run = new QHBoxLayout();
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
    layout->addWidget(camera_model1_groupbox);
    layout->addWidget(camera_model2_groupbox);
    layout->addWidget(stereo_calibration_groupbox);
    layout->addWidget(chessboard_groupbox);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Stereo Calibrate from Files");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  void StereoFileCalibrationDialog::SetOptions(Options options) {
    directory_edit1_->setText(QString::fromStdString(options.images_directory1));
    directory_edit2_->setText(QString::fromStdString(options.images_directory2));

    only_estimate_pose_checkbox_->setChecked(options.estimate_pose_only);

    camera_model1_->SetCameraModel(options.camera_model1);
    camera_model2_->SetCameraModel(options.camera_model2);

    if (options.initial_camera_parameters1.has_value()) {
      initial_parameters_1_->SetInitialParameters(ConvertInitialParameters(options.initial_camera_parameters1));
      use_initial_parameters_checkbox1_->setChecked(true);
    }
    else {
      initial_parameters_1_->SetInitialParameters({});
      use_initial_parameters_checkbox1_->setChecked(false);
    }

    if (options.initial_camera_parameters2.has_value()) {
      initial_parameters_2_->SetInitialParameters(ConvertInitialParameters(options.initial_camera_parameters2));
      use_initial_parameters_checkbox2_->setChecked(true);
    }
    else {
      initial_parameters_2_->SetInitialParameters({});
      use_initial_parameters_checkbox2_->setChecked(false);
    }

    calibration_target_options_->SetChessBoardTargetOptions(options.calibration_target_options);
  }

  StereoFileCalibrationDialog::Options StereoFileCalibrationDialog::GetOptions() {
    Options options;
    options.camera_model1 = camera_model1_->CameraModel();
    options.camera_model2 = camera_model2_->CameraModel();

    options.initial_camera_parameters1 = use_initial_parameters_checkbox1_->isChecked()
                                             ? ParseInitialParameters(initial_parameters_1_->InitialParameters())
                                             : std::optional<std::vector<double>>{};
    options.initial_camera_parameters2 = use_initial_parameters_checkbox2_->isChecked()
                                             ? ParseInitialParameters(initial_parameters_2_->InitialParameters())
                                             : std::optional<std::vector<double>>{};

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
    CameraModelType camera_model = camera_model1_->CameraModel();
    if (!ValidateParameters(initial_parameters_1_->InitialParameters(), camera_model, message) ||
        !ValidateParameters(initial_parameters_2_->InitialParameters(), camera_model, message)) {
      QMessageBox::information(this, "Validation Error", QString::fromStdString(message));
      return false;
    }

    if (only_estimate_pose_checkbox_->isChecked() &&
        (!use_initial_parameters_checkbox1_->isChecked() || !use_initial_parameters_checkbox2_->isChecked())) {
      QMessageBox::information(this, "Validation Error", "To estimate the relative pose only, intrinsics must be provided!");
      return false;
    }

    return true;
  }

  void StereoFileCalibrationDialog::ImportParameters(int camera) {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();
    if (path.empty()) {
      return;
    }

    ImportedParameters parameters = ImportedParameters::ImportFromYaml(path);

    if (camera == 0) {
      directory_edit1_->setText(QString::fromStdString(parameters.directory));
      camera_model1_->SetCameraModel(parameters.camera_model);
      initial_parameters_1_->SetInitialParameters(ConvertInitialParameters(parameters.camera_parameters));
      use_initial_parameters_checkbox1_->setChecked(true);
    }
    else {
      directory_edit2_->setText(QString::fromStdString(parameters.directory));
      camera_model2_->SetCameraModel(parameters.camera_model);
      initial_parameters_2_->SetInitialParameters(ConvertInitialParameters(parameters.camera_parameters));
      use_initial_parameters_checkbox2_->setChecked(true);
    }

    ChessboardFeatureExtractor::Options calibration_target_options;
    calibration_target_options.chessboard_columns = parameters.columns;
    calibration_target_options.chessboard_rows = parameters.rows;
    calibration_target_options.square_size = parameters.square_size;
    calibration_target_options_->SetChessBoardTargetOptions(calibration_target_options);
  }
}