
#include "main_window.h"

#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/calibrator.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/core/report.h"
#include "calibmar/readers/filesystem_reader.h"
#include "calibmar/readers/livestream_reader.h"
#include "calibmar/version.h"
#include "ui/dialogs/license_dialog.h"
#include "ui/dialogs/stereo_file_calibration_dialog.h"
#include "ui/dialogs/stream_calibration_dialog.h"
#include "ui/dialogs/test_widget_dialog.h"
#include "ui/utils/calibration_target_visualization.h"
#include "ui/utils/files_calibration_runner.h"
#include "ui/utils/livestream_calibration_runner.h"
#include "ui/utils/stereo_files_calibration_runner.h"
#include "ui/widgets/calibration_result_widget.h"
#include "ui/widgets/zoomable_scroll_area.h"

#include <QApplication>
#include <QFileDialog>
#include <QImageReader>
#include <QMenuBar>
#include <QMessageBox>
#include <QScreen>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <qlayout.h>
#include <regex>
#include <thread>

namespace calibmar {

  MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), scroll_area_(new QScrollArea(this)) {
    CreateActions();

    resize(QGuiApplication::primaryScreen()->availableSize() * 3 / 5);

    scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
    scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAsNeeded);
    setCentralWidget(scroll_area_);
    QWidget* content = new QWidget(this);
    main_layout_ = new QVBoxLayout(content);
    main_layout_->setSpacing(0);
    main_layout_->setAlignment(Qt::AlignTop);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setWidget(content);
    scrolled_to_bottom_ = true;
    QScrollBar* scrollbar = scroll_area_->verticalScrollBar();
    QObject::connect(scrollbar, &QScrollBar::rangeChanged, this, [this](int min, int max) {
      if (this->scrolled_to_bottom_) {
        this->scroll_area_->verticalScrollBar()->setValue(max);
      }
    });
    QObject::connect(scrollbar, &QScrollBar::valueChanged, this, [this](int value) {
      this->scrolled_to_bottom_ = this->scroll_area_->verticalScrollBar()->maximum() == value;
    });
  }

  // Starts a new live stream calibration
  void MainWindow::NewStreamCalibration() {
    StreamCalibrationDialog dialog(this);
    dialog.exec();

    if (dialog.result() != QDialog::DialogCode::Accepted) {
      return;
    }

    BeginNewCalibration();

    StreamCalibrationDialog::Options options = dialog.GetOptions();
    if (options.save_images_directory.has_value()) {
      last_directory_ = options.save_images_directory.value();
    }

    LiveStreamExtractionWidget* extraction_widget = new LiveStreamExtractionWidget(this);
    CalibrationWidget* calibration_widget = new CalibrationWidget(
        this, std::bind(&MainWindow::DisplayExtractionImage, this, std::placeholders::_1, std::placeholders::_2));
    calibration_widget->setVisible(false);
    main_layout_->addWidget(calibration_widget);
    main_layout_->addWidget(extraction_widget);

    std::unique_ptr<LivestreamCalibrationRunner> runner =
        std::make_unique<LivestreamCalibrationRunner>(calibration_widget, extraction_widget, options);
    worker_thread_.reset(new std::thread([this, runner = std::move(runner), calibration = calibration_.get()]() {
      bool success = runner->Run(*calibration);

      if (!success) {
        calibration_.reset();
      }

      QMetaObject::invokeMethod(this, [this]() { EndNewCalibration(); });
    }));

    worker_thread_->detach();
  }

  // Starts a new files calibration
  void MainWindow::NewFilesCalibration() {
    FileCalibrationDialog dialog(this);
    dialog.exec();

    if (dialog.result() != QDialog::DialogCode::Accepted) {
      return;
    }

    BeginNewCalibration();

    FileCalibrationDialog::Options options = dialog.GetOptions();
    last_directory_ = options.images_directory;

    CalibrationWidget* calibration_widget = new CalibrationWidget(
        this, std::bind(&MainWindow::DisplayExtractionImage, this, std::placeholders::_1, std::placeholders::_2));
    main_layout_->addWidget(calibration_widget);

    std::unique_ptr<FilesCalibrationRunner> runner = std::make_unique<FilesCalibrationRunner>(calibration_widget, options);
    worker_thread_.reset(new std::thread([this, runner = std::move(runner), calibration = calibration_.get()]() {
      bool success = runner->Run(*calibration);

      if (!success) {
        calibration_.reset();
      }

      QMetaObject::invokeMethod(this, [this]() { EndNewCalibration(); });
    }));

    worker_thread_->detach();
  }

  void MainWindow::NewStereoFilesCalibration() {
    StereoFileCalibrationDialog dialog(this);
    dialog.exec();

    if (dialog.result() != QDialog::DialogCode::Accepted) {
      return;
    }

    BeginNewCalibration();
    calibration_stereo_.reset(new Calibration());

    StereoFileCalibrationDialog::Options options = dialog.GetOptions();
    last_directory_ = options.images_directory2;
    last_directory_ = options.images_directory2;

    CalibrationWidget* calibration_widget = new CalibrationWidget(
        this, std::bind(&MainWindow::DisplayExtractionImage, this, std::placeholders::_1, std::placeholders::_2));
    main_layout_->addWidget(calibration_widget);

    std::unique_ptr<StereoFilesCalibrationRunner> runner =
        std::make_unique<StereoFilesCalibrationRunner>(calibration_widget, options);
    worker_thread_.reset(new std::thread(
        [this, runner = std::move(runner), calibration1 = calibration_.get(), calibration2 = calibration_stereo_.get()]() {
      bool success = runner->Run(*calibration1, *calibration2);

      if (!success) {
        calibration_.reset();
      }

      QMetaObject::invokeMethod(this, [this]() { EndNewCalibration(); });
    }));

    worker_thread_->detach();
  }

  // Saves current calibration at desired location
  void MainWindow::SaveCalibration() {
    if (!calibration_) {
      QMessageBox::information(this, "Save", "No current calibration!");
      return;
    }

    std::filesystem::path path(last_directory_);
    std::string default_file_name = "calibration.txt";

    if (std::filesystem::exists(path)) {
      path /= default_file_name;
    }
    else {
      path = default_file_name;
    }

    std::filesystem::path report_file(
        QFileDialog::getSaveFileName(this, "Save Calibration", QString::fromStdString(path.string()), "Text files (*.txt)")
            .toStdString());

    std::filesystem::path yaml_file = report_file;
    yaml_file.replace_extension("yaml");

    // if report doesnt exist, but yaml does, the dialog will not have asked for confirmation. So ask now.
    if (!std::filesystem::exists(report_file) && std::filesystem::exists(yaml_file)) {
      QMessageBox::StandardButton reply = QMessageBox::question(
          this, "Overwrite", QString::fromStdString(yaml_file.filename().string() + " already exists.\nDo you want to replace?"),
          QMessageBox::Ok | QMessageBox::Cancel);

      if (reply == QMessageBox::Cancel) {
        return;
      }
    }

    // do the same for stereo
    if (calibration_stereo_) {
      std::filesystem::path report_file_stereo = report_file;
      report_file_stereo.replace_extension();  // remove any potentially userdefined extensions
      report_file_stereo += "_stereo.txt";
      std::filesystem::path yaml_file_stereo = report_file_stereo;
      yaml_file_stereo.replace_extension("yaml");

      if (!std::filesystem::exists(report_file) &&
          (std::filesystem::exists(yaml_file_stereo) || std::filesystem::exists(report_file_stereo))) {
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, "Overwrite",
            QString::fromStdString(report_file_stereo.filename().string() + "/" + yaml_file_stereo.filename().string() +
                                   " already exists.\nDo you want to replace?"),
            QMessageBox::Ok | QMessageBox::Cancel);

        if (reply == QMessageBox::Cancel) {
          return;
        }
      }

      report::WriteCalibrationReport(report_file_stereo.string(), *calibration_stereo_);
      report::WriteCalibrationYaml(yaml_file_stereo.string(), *calibration_stereo_);
    }

    report::WriteCalibrationReport(report_file.string(), *calibration_);
    report::WriteCalibrationYaml(yaml_file.string(), *calibration_);
  }

  void MainWindow::About() {
    QMessageBox::about(this, "About Calibmar", QString::fromStdString("<b>Calibmar " + CALIBMAR_VERSION + "</b>"));
  }

  void MainWindow::CreateActions() {
    QMenu* fileMenu = menuBar()->addMenu("&File");
    calibration_files_action_ = fileMenu->addAction("&Calibrate from Files...", this, &MainWindow::NewFilesCalibration);
    calibration_stream_action_ = fileMenu->addAction("Calibrate from &Livestream...", this, &MainWindow::NewStreamCalibration);
    calibration_stereo_files_action_ =
        fileMenu->addAction("&Stereo Calibrate from Files...", this, &MainWindow::NewStereoFilesCalibration);
    ////
    // fileMenu->addAction("Test Widget Dialog", this, []() {
    //   TestWidgetDialog dialog;
    //   dialog.exec();
    // });
    ////
    fileMenu->addSeparator();
    calibration_save_action_ = fileMenu->addAction("&Save Calibration...", this, &MainWindow::SaveCalibration);
    calibration_save_action_->setEnabled(false);
    fileMenu->addSeparator();
    QAction* exitAct = fileMenu->addAction("E&xit", this, &QWidget::close);
    QMenu* helpMenu = menuBar()->addMenu("&Help");
    helpMenu->addAction("&Licenses", this, []() {
      LicenseDialog dialog;
      dialog.resize(500, 500);
      dialog.exec();
    });
    helpMenu->addAction("&About", this, &MainWindow::About);
  }

  // Called before beginning a new calibration. Enables/Disables context menus and resets current calibration.
  void MainWindow::BeginNewCalibration() {
    int num_widgets = main_layout_->count();
    for (int i = 0; i < num_widgets; i++) {
      QWidget* widget = main_layout_->itemAt(0)->widget();
      main_layout_->removeWidget(widget);
      delete widget;
    }

    calibration_files_action_->setEnabled(false);
    calibration_stereo_files_action_->setEnabled(false);
    calibration_stream_action_->setEnabled(false);
    calibration_save_action_->setEnabled(false);

    calibration_.reset(new Calibration());
    calibration_stereo_.reset();
  }

  // Called after a new calibration. Enables/Disables context menus.
  void MainWindow::EndNewCalibration() {
    calibration_files_action_->setEnabled(true);
    calibration_stereo_files_action_->setEnabled(true);
    calibration_stream_action_->setEnabled(true);
    calibration_save_action_->setEnabled(static_cast<bool>(calibration_));
  }

  // Callback. Used to display an extracted image in a separate dialog
  void MainWindow::DisplayExtractionImage(const std::string& image_name, const TargetVisualizer& target_visualizer) {
    if (!calibration_) {
      return;
    }
    if (!std::filesystem::exists(image_name)) {
      return;
    }
    std::unique_ptr<Pixmap> img = std::make_unique<Pixmap>();
    if (!img->Read(image_name)) {
      return;
    }

    // if known image add point visu
    for (auto& image : calibration_->Images()) {
      if (image.Name() == image_name) {
        target_visualizer.DrawTargetOnImage(*img, image);
        break;
      }
    }
    if (calibration_stereo_) {
      for (auto& image : calibration_stereo_->Images()) {
        if (image.Name() == image_name) {
          target_visualizer.DrawTargetOnImage(*img, image);
          break;
        }
      }
    }

    // display as dialog
    QDialog dialog(this);
    dialog.setGeometry(this->geometry());
    dialog.setWindowTitle(QString::fromStdString(image_name));
    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    ZoomableScrollArea* area = new ZoomableScrollArea(&dialog);
    ImageWidget* lbl = new ImageWidget(&dialog);
    lbl->SetImage(std::move(img));
    area->setWidget(lbl);
    layout->addWidget(area);
    dialog.exec();
  }
}
