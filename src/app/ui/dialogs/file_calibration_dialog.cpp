#include "file_calibration_dialog.h"

#include <calibmar/core/report.h>
#include <colmap/util/misc.h>
#include <filesystem>

namespace calibmar {

  FileCalibrationDialog::FileCalibrationDialog(QWidget* parent) : QDialog(parent) {
    // directory groupbox
    QGroupBox* directory_groupbox = new QGroupBox(this);
    directory_groupbox->setTitle("Image files directory");
    directory_edit_ = new QLineEdit(directory_groupbox);
    QPushButton* select_directory_button = new QPushButton(directory_groupbox);
    select_directory_button->setText("Browse");
    connect(select_directory_button, &QPushButton::released, this, [this]() {
      this->directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select calibration target image directory"));
    });
    QHBoxLayout* horizontal_layout_directory = new QHBoxLayout(directory_groupbox);
    horizontal_layout_directory->addWidget(directory_edit_);
    horizontal_layout_directory->addWidget(select_directory_button);

    // common options
    calibration_options_widget_ = new CommonCalibrationOptionsWidget(this);

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
    layout->addWidget(calibration_options_widget_);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Calibrate from Files");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  void FileCalibrationDialog::SetOptions(Options options) {
    directory_edit_->setText(QString::fromStdString(options.images_directory));

    CommonCalibrationOptionsWidget::Options calibration_options;
    calibration_options.calibration_target_options = options.calibration_target_options;
    calibration_options.camera_model = options.camera_model;
    calibration_options.housing_options = options.housing_calibration;
    calibration_options.initial_camera_parameters = options.initial_camera_parameters;
    calibration_options_widget_->SetOptions(calibration_options);
  }

  FileCalibrationDialog::Options FileCalibrationDialog::GetOptions() {
    CommonCalibrationOptionsWidget::Options calibration_options = calibration_options_widget_->GetOptions();
    Options options;
    options.camera_model = calibration_options.camera_model;
    options.housing_calibration = calibration_options.housing_options;
    options.initial_camera_parameters = calibration_options.initial_camera_parameters;
    options.calibration_target_options = calibration_options.calibration_target_options;
    options.images_directory = directory_edit_->text().toStdString();
    return options;
  }

  bool FileCalibrationDialog::Validate() {
    if (!std::filesystem::is_directory(directory_edit_->text().toStdString())) {
      QMessageBox::information(this, "Validation Error", "Image directory does not exist.");
      return false;
    }

    return calibration_options_widget_->Validate();
  }

  void FileCalibrationDialog::ImportParameters() {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();
    if (path.empty()) {
      return;
    }

    ImportedParameters p = ImportedParameters::ImportFromYaml(path);
    Options options;
    switch (p.calibration_target) {
      case CalibrationTargetType::Chessboard:
        options.calibration_target_options = ChessboardFeatureExtractor::Options{p.rows, p.columns, p.square_size};
        break;
      case CalibrationTargetType::Target3D:
        options.calibration_target_options = SiftFeatureExtractor::Options{};
        break;
      case CalibrationTargetType::Target3DAruco:
        options.calibration_target_options = ArucoSiftFeatureExtractor::Options{p.aruco_type, p.aruco_scale_factor, false};
        break;
      case CalibrationTargetType::ArucoGridBoard:
        options.calibration_target_options = ArucoBoardFeatureExtractor::Options{p.aruco_type,
                                                                                 ArucoGridOrigin::TopLeft,
                                                                                 ArucoGridDirection::Horizontal,
                                                                                 0,
                                                                                 p.columns,
                                                                                 p.rows,
                                                                                 p.square_size,
                                                                                 p.spacing,
                                                                                 1};
        break;
    }

    options.camera_model = p.camera_model;
    if (p.housing_model.has_value()) {
      options.housing_calibration = {p.housing_model.value(), p.housing_parameters};
    }
    options.images_directory = p.directory;
    options.initial_camera_parameters = p.camera_parameters;

    SetOptions(options);
  }
}