#include "livestream_calibration_runner.h"

#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/calibrators/opencv_calibration.h"
#include "calibmar/pose_suggestion/pose_suggestion.h"
#include "ui/utils/render.h"
#include "ui/widgets/calibration_result_widget.h"

#include <colmap/src/base/projection.h>
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace {
  void ProjectPoints(const colmap::Camera& camera, const std::vector<Eigen::Vector3d>& points3D, const Eigen::Vector4d& rotation,
                     const Eigen::Vector3d& translation, std::vector<Eigen::Vector2d>& points2D) {
    Eigen::Matrix3x4d proj_mat = colmap::ComposeProjectionMatrix(rotation, translation);

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation(0), rotation(1), rotation(2), rotation(3));
    Eigen::Affine3d trans = Eigen::Translation3d(translation) * q;

    for (const Eigen::Vector3d& point : points3D) {
      Eigen::Vector3d p = trans * point;
      Eigen::Vector2d test = camera.WorldToImage((trans * point).hnormalized());

      points2D.push_back(colmap::ProjectPointToImage(point, proj_mat, camera));
    }
  }

  void EstimateNextBestPose(calibmar::Calibration& calibration, int columns, int rows, double square_size,
                            const std::pair<int, int>& image_size, const std::vector<Eigen::Vector3d>& points3D,
                            std::vector<Eigen::Vector2d>& current_target_points) {
    Eigen::Vector4d rotation;
    Eigen::Vector3d translation;
    calibmar::pose_suggestion::EstimateNextBestPose(calibration, columns - 1, rows - 1, square_size, rotation, translation);
    current_target_points.clear();
    ProjectPoints(calibration.Camera(), points3D, rotation, translation, current_target_points);
  }
}

namespace calibmar {

  LivestreamCalibrationRunner::LivestreamCalibrationRunner(CalibrationWidget* calibration_widget,
                                                           LiveStreamExtractionWidget* extraction_widget,
                                                           const StreamCalibrationDialog::Options& dialog_options)
      : calibration_widget_(calibration_widget),
        extraction_widget_(extraction_widget),
        dialog_options_(dialog_options),
        new_extraction_(true),
        acquire_(false),
        run_extraction_(true),
        extract_(false),
        image_counter_(0) {
    ChessboardFeatureExtractor::Options extractor_options;
    extractor_options.chessboard_columns = dialog_options_.chessboard_columns;
    extractor_options.chessboard_rows = dialog_options_.chessboard_rows;
    extractor_options.square_size = dialog_options_.square_size;
    extractor_ = std::make_unique<ChessboardFeatureExtractor>(extractor_options);

    if (dialog_options_.acquisition_mode != StreamCalibrationDialog::AcquisitionMode::OnTimedDetection) {
      QMetaObject::invokeMethod(extraction_widget_, [this]() mutable {
        capture_button_ = new QPushButton("Capture [spacebar]");
        extraction_widget_->AddLiveModeWidget(capture_button_);
        capture_button_->grabKeyboard();
        capture_button_->connect(capture_button_, &QPushButton::released, [this]() { acquire_ = true; });
      });
    }

    std::function<void()> done = [this]() { run_extraction_ = false; };
    extraction_widget_->SetCompleteButtonCallback(done);
  }

  bool LivestreamCalibrationRunner::Run(Calibration& calibration) {
    if (dialog_options_.acquisition_mode == StreamCalibrationDialog::AcquisitionMode::OnSuggestedPose &&
        dialog_options_.housing_calibration.has_value()) {
      throw new std::runtime_error("Pose suggestion does not support housing calibration.");
    }
    // for image names
    time(&run_start_);
    image_counter_ = 0;

    LiveStreamImageReader::Options reader_options;
    reader_options.device_id = dialog_options_.device_index;
    LiveStreamImageReader reader(reader_options);
    reader.Open();
    std::pair<int, int> image_size{reader.ImagesWidth(), reader.ImagesHeight()};
    calibration.SetPoints3D(extractor_->Points3D());
    calibration.SetCalibrationTargetInfo("chessboard, " + std::to_string(dialog_options_.chessboard_columns) + ", " +
                                         std::to_string(dialog_options_.chessboard_rows) + ", " +
                                         std::to_string(dialog_options_.square_size));

    TargetTracker tracker({dialog_options_.chessboard_columns, dialog_options_.chessboard_rows}, image_size);

    // Start asyncronous extraction based on selected option
    std::unique_ptr<std::thread> extraction_thread;
    switch (dialog_options_.acquisition_mode) {
      case StreamCalibrationDialog::AcquisitionMode::OnButton:
        extraction_thread =
            std::make_unique<std::thread>(&LivestreamCalibrationRunner::RunBasicExtraction, this, std::ref(calibration));
        break;
      case StreamCalibrationDialog::AcquisitionMode::OnTimedDetection:
        extraction_thread = std::make_unique<std::thread>(&LivestreamCalibrationRunner::RunTimedExtraction, this,
                                                          std::ref(calibration), std::ref(tracker));
        break;
      case StreamCalibrationDialog::AcquisitionMode::OnSuggestedPose:
        extraction_thread =
            std::make_unique<std::thread>(&LivestreamCalibrationRunner::RunPoseSuggestionExtraction, this, std::ref(calibration),
                                          std::ref(tracker), dialog_options_.camera_model, std::ref(image_size));
        break;
    }

    while (run_extraction_ && reader.HasNext()) {
      // Display livestream images with full fps and update overlay if new extraction is availabe

      Image image;
      std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
      reader.Next(image, *pixmap.get());

      // INFO: all interaction between the extraction thread and this one
      // is guarded by the new_extraction_/extract_ signal, so no locking is needed
      if (new_extraction_) {
        // move image to extraction instead for next extraction
        current_pixmap_ = std::move(pixmap);

        current_draw_points_.clear();
        for (Eigen::Vector2d& point : current_extracted_points_) {
          current_draw_points_.push_back(cv::Point2f(point.x(), point.y()));
        }
        current_draw_target_points_.clear();
        for (Eigen::Vector2d& point : current_target_points_) {
          current_draw_target_points_.push_back(cv::Point2f(point.x(), point.y()));
        }

        new_extraction_ = false;
        extract_.Set();
        continue;
      }

      // draw currently extracted points and target
      render::DrawChessboardCorners(*pixmap, dialog_options_.chessboard_columns - 1, dialog_options_.chessboard_rows - 1,
                                    current_draw_points_);

      render::DrawChessboardGrid(*pixmap, dialog_options_.chessboard_columns - 1, dialog_options_.chessboard_rows - 1,
                                 current_draw_target_points_);

      // mirror?
      cv::flip(pixmap->Data(), pixmap->Data(), 1);

      // enqueue for drawing
      QMetaObject::invokeMethod(extraction_widget_,
                                [pixmap = std::move(pixmap), extraction_widget = extraction_widget_]() mutable {
        extraction_widget->SetLiveStreamImage(std::move(pixmap));
      });
    }

    // Begin calibrating
    reader.Close();
    // To allow the extraction thread to close
    extract_.Set();

    // Move extraction images to calibration UI
    QMetaObject::invokeMethod(calibration_widget_,
                              [calibration_widget = this->calibration_widget_, extraction_widget = this->extraction_widget_]() {
      extraction_widget->setVisible(false);

      std::vector<ExtractionImageWidget*> imgs = extraction_widget->RemoveExtractionImagesWidgets();
      for (auto widget : imgs) {
        calibration_widget->AddExtractionItem(widget);
      }

      delete extraction_widget;

      calibration_widget->setVisible(true);
      calibration_widget->StartCalibration();

      calibration_widget->update();
    });

    Calibrator::Options calibrator_options;
    if (dialog_options_.initial_camera_parameters.has_value()) {
      calibration.SetCamera(
          CameraModel::InitCamera(dialog_options_.camera_model, image_size, dialog_options_.initial_camera_parameters.value()));
      calibrator_options.use_intrinsics_guess = true;
    }
    calibrator_options.camera_model = dialog_options_.camera_model;
    calibrator_options.image_size = image_size;
    Calibrator calibrator(calibrator_options);

    try {
      extraction_thread->join();
      calibrator.Calibrate(calibration);
      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = this->calibration_widget_,
                                                      last_pixmap = std::move(last_pixmap_), &calibration]() mutable {
        calibration_widget->EndCalibration(new CalibrationResultWidget(calibration, std::move(last_pixmap)));
      });

      return true;
    }
    catch (std::exception& ex) {
      QMetaObject::invokeMethod(calibration_widget_,
                                [calibration_widget = this->calibration_widget_, message = std::string(ex.what())]() mutable {
        calibration_widget->EndCalibration(new CalibrationResultWidget(message));
      });

      return false;
    }
  }

  void LivestreamCalibrationRunner::DrawLivestreamImage(std::unique_ptr<Pixmap> pixmap, int columns, int rows) {
    cv::Mat& cornerMat = pixmap->Data();

    if (cornerMat.channels() == 1) {
      // reallocate in place, this is supported by opencv, the old one gets destroyed if refrence count == 0
      cv::cvtColor(cornerMat, cornerMat, cv::COLOR_GRAY2RGB);
    }

    if (!current_draw_points_.empty()) {
      cv::drawChessboardCorners(cornerMat, {columns - 1, rows - 1}, current_draw_points_, true);
    }

    QMetaObject::invokeMethod(extraction_widget_, [this, image = std::move(pixmap)]() mutable {
      extraction_widget_->SetLiveStreamImage(std::move(image));
    });
  }

  void LivestreamCalibrationRunner::RunBasicExtraction(Calibration& calibration) {
    while (run_extraction_) {
      extract_.Wait();
      Image image;
      if (extractor_->Extract(image, *current_pixmap_) == FeatureExtractor::Status::SUCCESS) {
        current_extracted_points_ = image.Points2D();
        ShowCapturePossible(true);
      }
      else {
        current_extracted_points_.clear();
        ShowCapturePossible(false);
      }

      if (acquire_ && current_extracted_points_.size() > 0) {
        Save(calibration, image, std::move(current_pixmap_));
      }

      acquire_ = false;
      new_extraction_ = true;
    }
  }

  void LivestreamCalibrationRunner::RunTimedExtraction(Calibration& calibration, TargetTracker& tracker) {
    TimerBarWidget* timer_widget;
    QMetaObject::invokeMethod(extraction_widget_, [&timer_widget, extraction_widget = extraction_widget_]() mutable {
      timer_widget = new TimerBarWidget();
      extraction_widget->AddLiveModeWidget(timer_widget);
    });

    std::chrono::milliseconds timer_duration = std::chrono::milliseconds(1500);
    bool timer_running = false;
    while (run_extraction_) {
      extract_.Wait();
      Image image;
      if (extractor_->Extract(image, *current_pixmap_) == FeatureExtractor::Status::SUCCESS) {
        current_extracted_points_ = image.Points2D();
        timer_widget->SetLabel("");
      }
      else {
        current_extracted_points_.clear();
        timer_widget->SetLabel("Looking for Target");
        // clear the target points to not block the timer start
        tracker.SetTargetPoints(current_extracted_points_);
      }

      tracker.Update(current_extracted_points_);

      // if not at target points during timer or if not stable -> stop timer
      if ((timer_running && !tracker.TargetPointsReached()) || !tracker.IsStable()) {
        timer_running = false;
        stable_detection_start_ = std::chrono::steady_clock::time_point::max();
        QMetaObject::invokeMethod(timer_widget, [timer_widget]() { timer_widget->StopTimer(); });
      }
      // else if timer not running && we are not at the last target pose -> start and remember this pose
      else if (!timer_running && !tracker.TargetPointsReached()) {
        timer_running = true;
        // Remember this pose as target points
        tracker.SetTargetPoints(current_extracted_points_);
        stable_detection_start_ = std::chrono::steady_clock::now();
        QMetaObject::invokeMethod(timer_widget, [timer_widget, timer_duration]() { timer_widget->StartTimer(timer_duration); });
      }
      // else if time up -> save image
      else if (timer_running && stable_detection_start_ + timer_duration < std::chrono::steady_clock::now()) {
        Save(calibration, image, std::move(current_pixmap_));
        // Update with empty points to force unstable
        tracker.Update(std::vector<Eigen::Vector2d>());
        QMetaObject::invokeMethod(timer_widget, [timer_widget]() { timer_widget->StopTimer(); });
      }

      new_extraction_ = true;
    }
  }

  void LivestreamCalibrationRunner::RunPoseSuggestionExtraction(Calibration& calibration, TargetTracker& tracker,
                                                                CameraModelType camera_model,
                                                                const std::pair<int, int>& image_size) {
    int image_width = image_size.first;
    int image_height = image_size.second;

    Calibrator::Options calibrator_options;
    calibrator_options.use_intrinsics_guess = true;
    calibrator_options.fast = true;
    Calibrator calibrator(calibrator_options);

    // Camera for init phase is always pinhole. Calibrating a complex distortion model with very few images can produce
    // extreme distortion coefficients. Projecting the target pose with those would create badly warped results.
    colmap::Camera init_camera;
    init_camera.InitializeWithName(CameraModel::CameraModels().at(CameraModelType::SimplePinholeCameraModel).model_name,
                                   1.2 * image_width, image_width, image_height);
    calibration.SetCamera(init_camera);

    // 3D point sets needed for opencv_calibration::CalibrateCamera
    std::vector<std::vector<Eigen::Vector3d>> points3D(1);
    for (auto& [id, point] : extractor_->Points3D()) {
      points3D[0].push_back(point);
    }

    // Estimate once to display first pose before first extraction (before chessboard is visible)
    EstimateNextBestPose(calibration, dialog_options_.chessboard_columns, dialog_options_.chessboard_rows,
                         dialog_options_.square_size, image_size, points3D[0], current_target_points_);

    double init_rms = std::numeric_limits<double>::max();
    bool in_init_phase = true;

    while (run_extraction_) {
      extract_.Wait();
      if (!run_extraction_) {
        // double check to prevent lengthy pose suggestion blocking the calibration
        return;
      }
      Image image;
      if (extractor_->Extract(image, *current_pixmap_) == FeatureExtractor::Status::SUCCESS) {
        current_extracted_points_ = image.Points2D();
        ShowCapturePossible(true);
      }
      else {
        current_extracted_points_.clear();
        ShowCapturePossible(false);
      }

      tracker.Update(current_extracted_points_);

      if (in_init_phase && !current_extracted_points_.empty()) {
        // during init phase calibrate on one image only to get valid focal length for pinhole camera
        // copy camera and only replace if the calibration rms is better than current camera
        colmap::Camera camera = calibration.Camera();
        Eigen::Vector4d rot = image.Rotation();
        Eigen::Vector3d trans = image.Translation();
        std::vector<Eigen::Vector4d*> rotations{&rot};
        std::vector<Eigen::Vector3d*> translations{&trans};
        double rms =
            opencv_calibration::CalibrateCamera(points3D, {image.Points2D()}, camera, true, true, rotations, translations);
        // ignore calibrated principle point to prevent "jumping" of the target pose
        // TODO: this could be done via CV_CALIB_FIX_PRINCIPLE
        camera.SetPrincipalPointX(image_width / 2.0);
        camera.SetPrincipalPointY(image_height / 2.0);
        if (rms < init_rms) {
          // use best rms camera
          init_rms = rms;
          calibration.SetCamera(camera);
        }

        EstimateNextBestPose(calibration, dialog_options_.chessboard_columns, dialog_options_.chessboard_rows,
                             dialog_options_.square_size, image_size, points3D[0], current_target_points_);
        tracker.SetTargetPoints(current_target_points_);
      }

      if (tracker.TargetPointsReached() || (acquire_ && !current_extracted_points_.empty())) {
        Save(calibration, image, std::move(current_pixmap_));

        if (in_init_phase && calibration.Images().size() >= 3) {
          // transition out of init phase, now use target camera model
          in_init_phase = false;
          colmap::Camera& init_camera = calibration.Camera();
          colmap::Camera calib_camera;
          calib_camera.InitializeWithName(CameraModel::CameraModels().at(dialog_options_.camera_model).model_name,
                                          init_camera.FocalLength(), init_camera.Width(), init_camera.Height());

          calibration.SetCamera(calib_camera);
        }

        if (!in_init_phase) {
          calibrator.Calibrate(calibration);
          EstimateNextBestPose(calibration, dialog_options_.chessboard_columns, dialog_options_.chessboard_rows,
                               dialog_options_.square_size, image_size, points3D[0], current_target_points_);
          tracker.SetTargetPoints(current_target_points_);
        }
      }

      acquire_ = false;
      new_extraction_ = true;
    }
  }

  // Add the image to the calibration. If configured also save to file.
  void calibmar::LivestreamCalibrationRunner::Save(Calibration& calibration, Image& image, std::unique_ptr<Pixmap> pixmap) {
    // Show shutter animation
    QMetaObject::invokeMethod(extraction_widget_,
                              [extraction_widget = extraction_widget_]() { extraction_widget->SignalImageAcquisition(); });

    // save a copy of the last image for the offset visualization
    last_pixmap_ = std::make_unique<Pixmap>(pixmap->Clone());

    if (dialog_options_.save_images_directory.has_value()) {
      char buf[sizeof "2022-22-22T22-22-22"];
      strftime(buf, sizeof buf, "%FT%H-%M-%S", localtime(&run_start_));
      std::string time_string(buf);
      std::filesystem::path file(time_string + "_" + std::to_string(image_counter_) + ".png");
      std::filesystem::path dir(dialog_options_.save_images_directory.value());
      std::string full_path((dir / file).string());
      cv::imwrite(full_path, pixmap->Data());
      image.SetName(full_path);
    }

    calibration.AddImage(image);
    // for image names
    image_counter_++;

    // display image in sidebar
    std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
    if (dialog_options_.save_images_directory.has_value()) {
      data->image_name = image.Name();
    }
    data->chessboard_columns = dialog_options_.chessboard_columns;
    data->chessboard_rows = dialog_options_.chessboard_rows;
    data->image = std::move(pixmap);
    data->points = image.Points2D();
    data->status = ExtractionImageWidget::Status::SUCCESS;
    QMetaObject::invokeMethod(extraction_widget_, [extraction_widget = extraction_widget_, data = std::move(data)]() mutable {
      extraction_widget->AddExtractionItem(std::move(data));
    });
  }

  // Notify the user on current status by displaying "Capture [spacebar]"/"Looking for Target"
  void calibmar::LivestreamCalibrationRunner::ShowCapturePossible(bool active) {
    QMetaObject::invokeMethod(capture_button_, [active, capture_button = capture_button_]() {
      QString text = active ? "Capture [spacebar]" : "Looking for Target";
      capture_button->setText(text);
      capture_button->setEnabled(active);
    });
  }
}