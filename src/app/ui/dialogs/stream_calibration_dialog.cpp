#include "stream_calibration_dialog.h"

#include "calibmar/readers/livestream_reader.h"

#include <filesystem>
#include <opencv2/imgproc.hpp>

namespace {
  int live_stream_height = 150;

  void InitAcquisitionModeMap(
      std::vector<std::pair<calibmar::StreamCalibrationDialog::AcquisitionMode, std::string>>& acquisition_modes) {
    acquisition_modes.push_back({calibmar::StreamCalibrationDialog::AcquisitionMode::OnTimedDetection, "Hands Free"});
    acquisition_modes.push_back({calibmar::StreamCalibrationDialog::AcquisitionMode::OnButton, "Button Press"});
    acquisition_modes.push_back({calibmar::StreamCalibrationDialog::AcquisitionMode::OnSuggestedPose, "Pose Suggestion"});
  }
}

namespace calibmar {

  StreamCalibrationDialog::StreamCalibrationDialog(QWidget* parent) : QDialog(parent) {
    InitAcquisitionModeMap(acquisition_modes_);
    // livestream groupbox
    QGroupBox* camera_groupbox = new QGroupBox(this);
    QVBoxLayout* livestream_layout = new QVBoxLayout(camera_groupbox);
    camera_groupbox->setTitle("Camera");
    // device index
    QLabel* device_index_label = new QLabel(camera_groupbox);
    device_index_label->setText("Device Index");
    device_index_ = new QSpinBox(this);
    device_index_->setRange(0, 10);
    device_index_->setSingleStep(1);
    device_index_->setValue(0);
    StartLiveStream(0);
    connect(device_index_, QOverload<int>::of(&QSpinBox::valueChanged), this, &StreamCalibrationDialog::StartLiveStream);
    QFormLayout* formLayout_index = new QFormLayout();
    formLayout_index->setWidget(0, QFormLayout::LabelRole, device_index_label);
    formLayout_index->setWidget(0, QFormLayout::FieldRole, device_index_);
    formLayout_index->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    // stream image
    image_widget_ = new ImageWidget(camera_groupbox);
    image_widget_->setMinimumHeight(live_stream_height);
    // acquisiton mode
    QLabel* mode_label = new QLabel(camera_groupbox);
    mode_label->setText("Acquisition Mode");
    mode_combobox_ = new QComboBox(camera_groupbox);
    for (auto& [mode, name] : acquisition_modes_) {
      mode_combobox_->addItem(QString::fromStdString(name));
    }
    mode_combobox_->setCurrentIndex(0);
    QFormLayout* formLayout_mode = new QFormLayout();
    formLayout_mode->setWidget(0, QFormLayout::LabelRole, mode_label);
    formLayout_mode->setWidget(0, QFormLayout::FieldRole, mode_combobox_);
    formLayout_mode->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);

    livestream_layout->addLayout(formLayout_index);
    livestream_layout->addWidget(image_widget_);
    livestream_layout->addLayout(formLayout_mode);
    livestream_layout->setAlignment(Qt::AlignTop);

    // save image directory
    QGroupBox* save_image_groupbox = new QGroupBox(this);
    save_image_groupbox->setTitle("Save Images to Directory");
    directory_edit_ = new QLineEdit(this);
    QPushButton* select_directory_button = new QPushButton(this);
    select_directory_button->setText("Browse");
    connect(select_directory_button, &QPushButton::released, this, [this]() {
      directory_edit_->setText(QFileDialog::getExistingDirectory(this, "Select calibration target image directory"));
    });
    save_images_checkbox_ = new QCheckBox(this);
    connect(save_images_checkbox_, &QCheckBox::stateChanged, this, [this, select_directory_button](int state) {
      directory_edit_->setEnabled(save_images_checkbox_->isChecked());
      select_directory_button->setEnabled(save_images_checkbox_->isChecked());
    });
    directory_edit_->setEnabled(save_images_checkbox_->isChecked());
    select_directory_button->setEnabled(save_images_checkbox_->isChecked());
    QHBoxLayout* horizontal_layout_directory = new QHBoxLayout(save_image_groupbox);
    horizontal_layout_directory->addWidget(save_images_checkbox_);
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
      if (!Validate()) {
        return;
      }
      this->accept();
    });
    horizontalLayout_run->addWidget(run_button, 0, Qt::AlignRight | Qt::AlignTop);

    // main layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(camera_groupbox);
    layout->addWidget(save_image_groupbox);
    layout->addWidget(calibration_options_widget_);
    layout->addLayout(horizontalLayout_run);
    setWindowTitle("Chessboard Calibration");
    layout->setSizeConstraint(QLayout::SetMinimumSize);
  }

  StreamCalibrationDialog::Options StreamCalibrationDialog::GetOptions() {
    Options options;
    options.camera_model = calibration_options_widget_->CameraModel();
    options.chessboard_columns = calibration_options_widget_->ChessboardColumns();
    options.chessboard_rows = calibration_options_widget_->ChessboardRows();
    options.housing_calibration = calibration_options_widget_->HousingOptions();
    options.initial_camera_parameters = calibration_options_widget_->InitialCameraParameters();
    options.square_size = calibration_options_widget_->SquareSize();
    options.device_index = device_index_->value();
    options.acquisition_mode = acquisition_modes_.at(mode_combobox_->currentIndex()).first;
    options.save_images_directory = save_images_directory_;
    return options;
  }

  void StreamCalibrationDialog::done(int r) {
    CloseLiveStream();
    QDialog::done(r);
  }

  void StreamCalibrationDialog::StartLiveStream(int index) {
    cancel_ = true;

    if (worker_thread_) {
      worker_thread_->join();
    }

    valid_device_ = false;
    cancel_ = false;

    worker_thread_.reset(new std::thread([this, index]() {
      LiveStreamImageReader reader(LiveStreamImageReader::Options{index});

      std::unique_ptr<Pixmap> pixmap;
      bool error = false;

      while (!this->cancel_ && reader.HasNext()) {
        pixmap.reset(new Pixmap());

        try {
          Image image;
          ImageReader::Status status = reader.Next(image, *pixmap.get());

          error = status != ImageReader::Status::SUCCESS;
        }
        catch (std::exception& ex) {
          error = true;
        }

        if (error) {
          cv::Mat error_image(live_stream_height, live_stream_height, CV_8UC1, cv::Scalar(0));
          cv::putText(error_image, "No device at " + std::to_string(index), cv::Point(5, live_stream_height / 2 + 5),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1);
          pixmap->Assign(error_image);
          this->cancel_ = true;
        }
        else if (!valid_device_) {
          valid_device_ = true;
        }

        QMetaObject::invokeMethod(this, [this, pixmap = std::move(pixmap)]() mutable {
          this->image_widget_->SetImage(std::move(pixmap));
          this->image_widget_->update();
        });
      }
    }));
  }

  void StreamCalibrationDialog::CloseLiveStream() {
    cancel_ = true;
    if (worker_thread_) {
      worker_thread_->join();
    }
  }

  bool StreamCalibrationDialog::Validate() {
    if (!valid_device_) {
      QMessageBox::information(this, "Validation Error", "Device not found.");
      return false;
    }
    if (save_images_checkbox_->isChecked()) {
      if (!std::filesystem::is_directory(directory_edit_->text().toStdString())) {
        QMessageBox::information(this, "Validation Error", "Image directory does not exist.");
        save_images_directory_ = {};
        return false;
      }
      else {
        save_images_directory_ = directory_edit_->text().toStdString();
      }
    }
    else {
      save_images_directory_ = {};
    }

    if (!calibration_options_widget_->Validate()) {
      return false;
    }

    if (calibration_options_widget_->HousingOptions().has_value() &&
        acquisition_modes_.at(mode_combobox_->currentIndex()).first == AcquisitionMode::OnSuggestedPose) {
      QMessageBox::information(this, "Validation Error",
                               "Acquisition mode Pose Suggestion does not support housing calibration.");
      return false;
    }

    return true;
  }

  void calibmar::StreamCalibrationDialog::ImportParameters() {
    std::string path =
        QFileDialog::getOpenFileName(this, "Import Parameters", QString(), "Calibration YAML (*.yaml *.yml)").toStdString();

    ImportedParameters p = ImportedParameters::ImportFromYaml(path);
    directory_edit_->setText(QString::fromStdString(p.directory));
    calibration_options_widget_->SetImportedParameters(p);
  }
}
