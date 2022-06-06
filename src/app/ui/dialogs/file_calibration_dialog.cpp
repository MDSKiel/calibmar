#include "file_calibration_dialog.h"

#include <calibmar/core/report.h>
#include <colmap/src/util/misc.h>
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
    setWindowTitle("Chessboard Calibration");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  FileCalibrationDialog::Options FileCalibrationDialog::GetOptions() {
    Options options;
    options.camera_model = calibration_options_widget_->CameraModel();
    options.housing_calibration = calibration_options_widget_->HousingOptions();
    options.initial_camera_parameters = calibration_options_widget_->InitialCameraParameters();
    options.images_directory = directory_edit_->text().toStdString();
    options.chessboard_columns = calibration_options_widget_->ChessboardColumns();
    options.chessboard_rows = calibration_options_widget_->ChessboardRows();
    options.square_size = calibration_options_widget_->SquareSize();
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

    ImportedParameters p = ImportedParameters::ImportFromYaml(path);
    directory_edit_->setText(QString::fromStdString(p.directory));
    calibration_options_widget_->SetImportedParameters(p);
  }
}