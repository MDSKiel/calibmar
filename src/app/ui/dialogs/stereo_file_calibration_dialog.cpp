#include "stereo_file_calibration_dialog.h"

#include <calibmar/core/report.h>
#include <colmap/util/misc.h>
#include <filesystem>

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

    // common options
    camera_model_selector_ = new CameraModelSelectorWidget(this);

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
    layout->addWidget(camera_model_selector_);
    layout->addWidget(chessboard_groupbox);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Stereo Calibrate from Files");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  StereoFileCalibrationDialog::Options StereoFileCalibrationDialog::GetOptions() {
    Options options;
    options.camera_model = camera_model_selector_->CameraModel();
    options.initial_camera_parameters = camera_model_selector_->InitialCameraParameters();

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
    if (!camera_model_selector_->Validate(message)) {
      QMessageBox::information(this, "Validation Error", QString::fromStdString(message));
      return false;
    }

    return true;
  }

  void StereoFileCalibrationDialog::ImportParameters() {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();
    
    ImportedParameters parameters = ImportedParameters::ImportFromYaml(path);

    // directories
    directory_edit1_->setText(QString::fromStdString(parameters.directory));
    directory_edit2_->setText(QString::fromStdString(parameters.directory));

    // camera model/params
    camera_model_selector_->SetCameraModel(parameters.camera_model);
    std::optional<std::string> params;
    if (parameters.camera_parameters.size() > 0) {
      params = colmap::VectorToCSV<double>(parameters.camera_parameters);
    }
    camera_model_selector_->SetInitialCameraParameters(params);

    // chessboard target
    ChessboardFeatureExtractor::Options target_options;
    target_options.chessboard_columns = parameters.chessboard_columns;
    target_options.chessboard_rows = parameters.chessboard_rows;
    target_options.square_size = parameters.square_size;
    calibration_target_options_->SetChessBoardTargetOptions(target_options);
  }
}