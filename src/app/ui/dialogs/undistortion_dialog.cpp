#include "undistortion_dialog.h"

#include "calibmar/core/report.h"
#include "calibmar/core/undistort.h"
#include "calibmar/readers/filesystem_reader.h"
#include <colmap/scene/camera.h>
#include <colmap/util/misc.h>

#include <filesystem>
#include <thread>

namespace {
  void UndistortImages(calibmar::FilesystemImageReader& reader, colmap::Camera& distorted_camera,
                       calibmar::CancellationToken& cancel, std::filesystem::path output_dir) {
    using namespace calibmar;
    calibmar::Image image;
    while (reader.HasNext() && !cancel) {
      std::unique_ptr<Pixmap> distorted_pixmap = std::make_unique<Pixmap>();
      Pixmap undistorted_pixmap;

      if (reader.Next(image, *distorted_pixmap) != FilesystemImageReader::Status::SUCCESS) {
        continue;
      }
      colmap::Camera undistorted_camera;
      undistort::UndistortImage(std::move(distorted_pixmap), distorted_camera, undistorted_pixmap, undistorted_camera);

      std::filesystem::path file_name(image.Name());
      file_name = (file_name.stem() += "_u") += file_name.extension();
      std::filesystem::path output_file = output_dir / file_name;
      undistorted_pixmap.Write(output_file.string());
    }
  }
}

namespace calibmar {

  UndistortionDialog::UndistortionDialog(QWidget* parent)
      : QDialog(parent), cancellation_flag_(false), cancel_(cancellation_flag_) {
    // input directory groupbox
    QGroupBox* input_directory = new QGroupBox(this);
    input_directory->setTitle("Input image files directory");
    input_directory_edit_ = new QLineEdit(input_directory);
    QPushButton* input_directory_button = new QPushButton(input_directory);
    input_directory_button->setText("Browse");
    connect(input_directory_button, &QPushButton::released, this, [this]() {
      this->input_directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select input images directory"));
    });
    QHBoxLayout* horizontal_layout_input = new QHBoxLayout(input_directory);
    horizontal_layout_input->addWidget(input_directory_edit_);
    horizontal_layout_input->addWidget(input_directory_button);
    // output directory groupbox
    QGroupBox* output_directory = new QGroupBox(this);
    output_directory->setTitle("Output image files directory");
    output_directory_edit_ = new QLineEdit(output_directory);
    QPushButton* output_directory_button = new QPushButton(output_directory);
    output_directory_button->setText("Browse");
    connect(output_directory_button, &QPushButton::released, this, [this]() {
      this->output_directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select output images directory"));
    });
    QHBoxLayout* horizontal_layout_output = new QHBoxLayout(output_directory);
    horizontal_layout_output->addWidget(output_directory_edit_);
    horizontal_layout_output->addWidget(output_directory_button);

    // camera
    camera_model_edit_ = new CameraModelSelectorWidget(this);

    // import button
    QHBoxLayout* horizontalLayout_run = new QHBoxLayout();
    QPushButton* import_button = new QPushButton(this);
    import_button->setText("Import...");
    connect(import_button, &QPushButton::released, this, [this]() { ImportParameters(); });
    horizontalLayout_run->addWidget(import_button, 0, Qt::AlignLeft | Qt::AlignTop);

    // run button
    QPushButton* run_button = new QPushButton(this);
    run_button->setText("Start");
    run_button->setDefault(true);
    connect(run_button, &QPushButton::released, this, [this]() {
      setEnabled(false);
      if (!StartUndistortingImages()) {
        // If undistorting didnt start (some input error) reenable dialog
        setEnabled(true);
      }
    });

    horizontalLayout_run->addWidget(run_button, 0, Qt::AlignRight | Qt::AlignTop);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(input_directory);
    layout->addWidget(output_directory);
    layout->addWidget(camera_model_edit_);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Undistort Images");

    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  void UndistortionDialog::ImportParameters() {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();
    if (path.empty()) {
      return;
    }

    ImportedParameters p = ImportedParameters::ImportFromYaml(path);
    camera_model_edit_->SetCameraModel(p.camera_model);
    camera_model_edit_->SetInitialCameraParameters(p.camera_parameters);
  }

  bool UndistortionDialog::StartUndistortingImages() {
    // validate inputs
    std::filesystem::path input_dir(input_directory_edit_->text().toStdString());
    std::filesystem::path output_dir(output_directory_edit_->text().toStdString());
    if (!std::filesystem::is_directory(input_dir)) {
      QMessageBox::critical(this, "", "Input directory does not exist!");
      return false;
    }
    if (!std::filesystem::is_directory(output_dir)) {
      QMessageBox::critical(this, "", "Ouput directory does not exist!");
      return false;
    }
    std::string msg;
    if (!camera_model_edit_->Validate(msg)) {
      QMessageBox::critical(this, "", QString::fromStdString(msg));
      return false;
    }
    if (!camera_model_edit_->InitialCameraParameters().has_value()) {
      QMessageBox::critical(this, "", "Camera parameters required!");
      return false;
    }
    if (!std::filesystem::is_empty(output_dir)) {
      if (QMessageBox::warning(
              this, "",
              "Output directory is not empty. Potentially duplicate files will be overwritten (input filenames are "
              "suffixed with '_u')",
              QMessageBox::Ok, QMessageBox::Cancel) != QMessageBox::Ok) {
        return false;
      }
    }
    int width, height;
    FilesystemImageReader::Options options;
    options.image_directory = input_dir.string();
    options.image_read_mode = Pixmap::ReadMode::COLOR_AS_SOURCE;
    std::unique_ptr<FilesystemImageReader> reader = std::make_unique<FilesystemImageReader>(options);

    try {
      width = reader->ImagesWidth();
      height = reader->ImagesHeight();
    }
    catch (std::runtime_error) {
      QMessageBox::critical(this, "", "Could not read any images from the input directory!");
      return false;
    }

    std::unique_ptr<colmap::Camera> camera = std::make_unique<colmap::Camera>(CameraModel::InitCamera(
        camera_model_edit_->CameraModel(), {width, height}, camera_model_edit_->InitialCameraParameters().value()));

    if (camera->IsUndistorted()) {
      QMessageBox::critical(this, "", "Provided camera is not distorted (distortion values are too small)!");
      return false;
    }

    // Show busy dialog with cancel option
    QDialog* progress_dialog = new QDialog(this);
    progress_dialog->setWindowTitle("Undistorting...");
    QVBoxLayout* layout = new QVBoxLayout(progress_dialog);
    QProgressBar* bar = new QProgressBar(progress_dialog);
    bar->setTextVisible(false);
    bar->setMaximum(0);
    bar->setMinimum(0);
    bar->show();
    bar->setOrientation(Qt::Orientation::Horizontal);
    QPushButton* button = new QPushButton("Cancel", progress_dialog);
    connect(button, &QPushButton::pressed, [flag = &cancellation_flag_, button]() {
      flag->store(true);
      button->setDisabled(true);
    });
    layout->addWidget(bar);
    layout->addWidget(button);
    progress_dialog->show();
    progress_dialog->setFixedSize(progress_dialog->size());

    // Start the runner which will close the dialog after being finished
    runner_ = std::make_unique<std::thread>(
        [dialog = this, progress_dialog, reader = std::move(reader), camera = std::move(camera), output_dir]() {
      std::vector<std::unique_ptr<std::thread>> runners;
      FilesystemImageReader& reader_ref = *reader;
      colmap::Camera& camera_ref = *camera;
      for (size_t i = 0; i < std::thread::hardware_concurrency(); i++) {
        std::unique_ptr<std::thread> runner = std::make_unique<std::thread>(
            UndistortImages, std::ref(reader_ref), std::ref(camera_ref), std::ref(dialog->cancel_), output_dir);
        runners.push_back(std::move(runner));
      }

      for (auto&& runner : runners) {
        runner->join();
      }

      QMetaObject::invokeMethod(dialog, [dialog, progress_dialog]() {
        progress_dialog->close();

        // ensure the runner can be cleaned up on close
        dialog->runner_->join();

        dialog->close();
      });
    });

    return true;
  }
}