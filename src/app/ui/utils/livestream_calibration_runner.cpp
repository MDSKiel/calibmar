#include "livestream_calibration_runner.h"

#include "calibmar/calibrators/calibrator3D.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/calibrators/opencv_calibration.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/pose_suggestion/pose_suggestion.h"
#include "ui/utils/render.h"
#include "ui/widgets/calibration_result_widget.h"
#include "ui/widgets/capture_button.h"

#include <colmap/mvs/image.h>
#include <filesystem>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace {
  void ProjectPoints(const colmap::Camera& camera, const std::vector<Eigen::Vector3d>& points3D, const Eigen::Vector4d& rotation,
                     const Eigen::Vector3d& translation, std::vector<Eigen::Vector2d>& points2D) {
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation(0), rotation(1), rotation(2), rotation(3));
    Eigen::Affine3d trans = Eigen::Translation3d(translation) * q;

    for (const Eigen::Vector3d& point : points3D) {
      points2D.push_back(camera.ImgFromCam((trans * point).hnormalized()));
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

  void SetupChessboardCalibration(calibmar::ChessboardFeatureExtractor::Options& chessboard_options,
                                  const calibmar::StreamCalibrationDialog::Options& options, calibmar::Calibration& calibration,
                                  std::pair<int, int> image_size, std::unique_ptr<calibmar::FeatureExtractor>& extractor,
                                  std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer,
                                  std::unique_ptr<calibmar::Calibrator>& calibrator,
                                  std::unique_ptr<calibmar::TargetTracker>& target_tracker) {
    using namespace calibmar;
    chessboard_options.fast = true;
    std::unique_ptr<ChessboardFeatureExtractor> chessboard_extractor =
        std::make_unique<ChessboardFeatureExtractor>(chessboard_options);
    calibration.SetPoints3D(chessboard_extractor->Points3D());
    calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(chessboard_options));

    if (options.housing_calibration.has_value()) {
      // housing calibration
      HousingCalibrator::Options calibrator_options;
      calibrator_options.camera_model = options.camera_model;
      calibrator_options.image_size = image_size;
      calibrator_options.estimate_initial_dome_offset =
          options.housing_calibration.value().first == HousingInterfaceType::DoubleLayerSphericalRefractive;
      calibrator_options.housing_interface = options.housing_calibration.value().first;
      calibrator_options.camera_params = options.initial_camera_parameters.value();
      calibrator_options.initial_housing_params = options.housing_calibration.value().second;
      calibrator_options.pattern_cols_rows = {chessboard_options.chessboard_columns, chessboard_options.chessboard_rows};
      calibrator = std::make_unique<HousingCalibrator>(calibrator_options);
    }
    else {
      // camera clibration
      BasicCalibrator::Options calibrator_options;
      if (options.initial_camera_parameters.has_value()) {
        calibration.SetCamera(
            CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters.value()));
        calibrator_options.use_intrinsics_guess = true;
      }
      else {
        calibrator_options.camera_model = options.camera_model;
        calibrator_options.image_size = image_size;
      }
      calibrator = std::make_unique<BasicCalibrator>(calibrator_options);
    }
    extractor = std::move(chessboard_extractor);
    target_visualizer =
        std::make_unique<ChessboardTargetVisualizer>(chessboard_options.chessboard_columns, chessboard_options.chessboard_rows);
    target_tracker = std::make_unique<ChessboardTargetTracker>(
        std::pair<int, int>{chessboard_options.chessboard_columns, chessboard_options.chessboard_rows}, image_size);
  }

  void Setup3DTargetCalibration(calibmar::Target3DTargetOptionsWidget::Options target3d_options,
                                const calibmar::StreamCalibrationDialog::Options& options, calibmar::Calibration& calibration,
                                std::pair<int, int> image_size, std::unique_ptr<calibmar::FeatureExtractor>& extractor,
                                std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer,
                                std::unique_ptr<calibmar::Calibrator>& calibrator,
                                std::unique_ptr<calibmar::TargetTracker>& target_tracker) {
    using namespace calibmar;

    calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(target3d_options));

    colmap::Camera camera;
    if (options.initial_camera_parameters.has_value()) {
      camera = CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters.value());
    }
    else {
      double focal_length = 1.2 * std::max(image_size.first, image_size.second);
      camera = colmap::Camera::CreateFromModelName(colmap::kInvalidCameraId,
                                                   CameraModel::CameraModels().at(options.camera_model).model_name, focal_length,
                                                   image_size.first, image_size.second);
    }

    calibration.SetCamera(camera);

    Calibrator3D::Options calibrator_options;
    bool use_aruco = std::holds_alternative<ArucoSiftFeatureExtractor::Options>(target3d_options);
    calibrator_options.contains_arucos = use_aruco;
    if (options.initial_camera_parameters.has_value()) {
      calibration.SetCamera(CameraModel::InitCamera(options.camera_model, image_size, options.initial_camera_parameters.value()));
      calibrator_options.use_intrinsics_guess = true;
    }
    else {
      calibrator_options.camera_model = options.camera_model;
      calibrator_options.image_size = image_size;
    }

    double aruco_mask_factor = 1;
    if (use_aruco) {
      ArucoSiftFeatureExtractor::Options options = std::get<ArucoSiftFeatureExtractor::Options>(target3d_options);
      aruco_mask_factor = options.masking_scale_factor;
      options.only_extract_aruco = true;
      extractor = std::make_unique<ArucoSiftFeatureExtractor>(options);
    }
    else {
      // this is disabled
      SiftFeatureExtractor::Options options = std::get<SiftFeatureExtractor::Options>(target3d_options);
      extractor = std::make_unique<SiftFeatureExtractor>(options);
    }

    calibrator = std::make_unique<Calibrator3D>(calibrator_options);
    target_visualizer = std::make_unique<Target3DTargetVisualizer>(use_aruco, aruco_mask_factor);
    target_tracker = std::make_unique<Aruco3DTargetTracker>(image_size, 0.05);
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
    if (dialog_options_.acquisition_mode != StreamCalibrationDialog::AcquisitionMode::OnTimedDetection) {
      QMetaObject::invokeMethod(extraction_widget_, [this]() mutable {
        capture_button_ = new CaptureButton("Capture [spacebar]");
        // such that the button can capture space bar
        QApplication::instance()->installEventFilter(capture_button_);
        extraction_widget_->AddLiveModeWidget(capture_button_);
        capture_button_->connect(capture_button_, &QPushButton::released, [this]() { acquire_ = true; });
      });
    }

    std::function<void()> done = [this]() { run_extraction_ = false; };
    extraction_widget_->SetCompleteButtonCallback(done);
  }

  bool LivestreamCalibrationRunner::Run(Calibration& calibration) {
    bool is_chessboard_calibration =
        std::holds_alternative<ChessboardFeatureExtractor::Options>(dialog_options_.calibration_target_options);

    if (dialog_options_.acquisition_mode == StreamCalibrationDialog::AcquisitionMode::OnSuggestedPose &&
        dialog_options_.housing_calibration.has_value()) {
      throw std::runtime_error("Pose suggestion does not support housing calibration.");
    }
    if (dialog_options_.acquisition_mode == StreamCalibrationDialog::AcquisitionMode::OnSuggestedPose &&
        !is_chessboard_calibration) {
      throw std::runtime_error("Pose suggestion only supports chessboard target calibration.");
    }

    // for image names
    time(&run_start_);
    image_counter_ = 0;

    LiveStreamImageReader::Options reader_options;
    reader_options.device_id = dialog_options_.device_index;
    LiveStreamImageReader reader(reader_options);
    reader.Open();
    std::pair<int, int> image_size{reader.ImagesWidth(), reader.ImagesHeight()};

    std::unique_ptr<Calibrator> calibrator;
    std::unique_ptr<TargetTracker> target_tracker;

    if (is_chessboard_calibration) {
      ChessboardFeatureExtractor::Options& chessboard_options =
          std::get<ChessboardFeatureExtractor::Options>(dialog_options_.calibration_target_options);
      SetupChessboardCalibration(chessboard_options, dialog_options_, calibration, image_size, extractor_, target_visualizer_,
                                 calibrator, target_tracker);
    }
    else {
      Target3DTargetOptionsWidget::Options target3d_options;
      if (std::holds_alternative<ArucoSiftFeatureExtractor::Options>(dialog_options_.calibration_target_options)) {
        target3d_options = std::get<ArucoSiftFeatureExtractor::Options>(dialog_options_.calibration_target_options);
      }
      else {
        target3d_options = std::get<SiftFeatureExtractor::Options>(dialog_options_.calibration_target_options);
      }

      Setup3DTargetCalibration(target3d_options, dialog_options_, calibration, image_size, extractor_, target_visualizer_,
                               calibrator, target_tracker);
    }

    // Start asyncronous extraction based on selected option
    std::unique_ptr<std::thread> extraction_thread;
    switch (dialog_options_.acquisition_mode) {
      case StreamCalibrationDialog::AcquisitionMode::OnButton:
        extraction_thread =
            std::make_unique<std::thread>(&LivestreamCalibrationRunner::RunBasicExtraction, this, std::ref(calibration));
        break;
      case StreamCalibrationDialog::AcquisitionMode::OnTimedDetection:
        extraction_thread = std::make_unique<std::thread>(&LivestreamCalibrationRunner::RunTimedExtraction, this,
                                                          std::ref(calibration), std::ref(*target_tracker));
        break;
      case StreamCalibrationDialog::AcquisitionMode::OnSuggestedPose:
        extraction_thread =
            std::make_unique<std::thread>(&LivestreamCalibrationRunner::RunPoseSuggestionExtraction, this, std::ref(calibration),
                                          std::ref(*target_tracker), dialog_options_.camera_model, std::ref(image_size));
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
        // clone image to extraction so it can extract in parallel
        current_pixmap_ = std::make_unique<Pixmap>(pixmap.get()->Clone());

        // copy and replace points
        current_draw_image_ = currently_extracted_image_;
        current_draw_target_points_ = current_target_points_;

        new_extraction_ = false;
        extract_.Set();
      }

      // draw currently extracted points and target
      target_visualizer_->DrawTargetOnImage(*pixmap, current_draw_image_);

      if (is_chessboard_calibration) {
        static_cast<ChessboardTargetVisualizer*>(target_visualizer_.get())
            ->DrawTargetPoseOnImage(*pixmap, current_draw_target_points_);
      }

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

    try {
      extraction_thread->join();

      // Currently a bit ugly: Move the target visualizer reference to the calibration widget, so it stays alive for the
      // extraction widgets livetime, which hold a reference to it
      calibration_widget_->SetTargetVisualizer(std::move(target_visualizer_));

      // Move extraction images to calibration UI
      // Also clean up extraction_widget (this has to be done after the extraction_thread has joined, since it accesses the
      // widget)
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

      calibrator->Calibrate(calibration);

      std::shared_ptr<colmap::Reconstruction> reconstruction;
      if (!is_chessboard_calibration) {
        reconstruction = static_cast<Calibrator3D*>(calibrator.get())->Reconstruction();
      }

      QMetaObject::invokeMethod(calibration_widget_,
                                [calibration_widget = this->calibration_widget_, last_pixmap = std::move(last_pixmap_),
                                 &calibration, reconstruction]() mutable {
        calibration_widget->EndCalibration(new CalibrationResultWidget(calibration, std::move(last_pixmap), reconstruction));
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

    if (!current_draw_image_.Points2D().empty()) {
      target_visualizer_->DrawTargetOnImage(*pixmap, current_draw_image_);
    }

    QMetaObject::invokeMethod(extraction_widget_, [this, image = std::move(pixmap)]() mutable {
      extraction_widget_->SetLiveStreamImage(std::move(image));
    });
  }

  void LivestreamCalibrationRunner::RunBasicExtraction(Calibration& calibration) {
    while (run_extraction_) {
      extract_.Wait();
      if (extractor_->Extract(currently_extracted_image_, *current_pixmap_) == FeatureExtractor::Status::SUCCESS) {
        ShowCapturePossible(true);
      }
      else {
        currently_extracted_image_.SetPoints2D({});
        ShowCapturePossible(false);
      }

      if (acquire_ && currently_extracted_image_.Points2D().size() > 0) {
        Save(calibration, currently_extracted_image_, std::move(current_pixmap_));
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
      if (extractor_->Extract(currently_extracted_image_, *current_pixmap_) == FeatureExtractor::Status::SUCCESS) {
        timer_widget->SetLabel("");
      }
      else {
        currently_extracted_image_.SetPoints2D({});
        timer_widget->SetLabel("Looking for Target");
        // clear the target points to not block the timer start
        tracker.SetTargetPoints(currently_extracted_image_.Points2D());
      }

      tracker.Update(currently_extracted_image_.Points2D());

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
        tracker.SetTargetPoints(currently_extracted_image_.Points2D());
        stable_detection_start_ = std::chrono::steady_clock::now();
        QMetaObject::invokeMethod(timer_widget, [timer_widget, timer_duration]() { timer_widget->StartTimer(timer_duration); });
      }
      // else if time up -> save image
      else if (timer_running && stable_detection_start_ + timer_duration < std::chrono::steady_clock::now()) {
        Save(calibration, currently_extracted_image_, std::move(current_pixmap_));
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

    BasicCalibrator::Options calibrator_options;
    calibrator_options.use_intrinsics_guess = true;
    calibrator_options.fast = true;
    BasicCalibrator calibrator(calibrator_options);

    // Camera for init phase is always pinhole. Calibrating a complex distortion model with very few images can produce
    // extreme distortion coefficients. Projecting the target pose with those would create badly warped results.
    colmap::Camera init_camera = colmap::Camera::CreateFromModelName(
        colmap::kInvalidCameraId, CameraModel::CameraModels().at(CameraModelType::SimplePinholeCameraModel).model_name,
        1.2 * image_width, image_width, image_height);
    calibration.SetCamera(init_camera);

    // Pose Suggestion is only allowed for chessboard target
    ChessboardFeatureExtractor* chessboard_extractor = static_cast<ChessboardFeatureExtractor*>(extractor_.get());
    ChessboardFeatureExtractor::Options chessboard_options =
        std::get<ChessboardFeatureExtractor::Options>(dialog_options_.calibration_target_options);

    // 3D point sets needed for opencv_calibration::CalibrateCamera
    std::vector<std::vector<Eigen::Vector3d>> points3D(1);
    for (auto& [id, point] : chessboard_extractor->Points3D()) {
      points3D[0].push_back(point);
    }

    // Estimate once to display first pose before first extraction (before chessboard is visible)
    EstimateNextBestPose(calibration, chessboard_options.chessboard_columns, chessboard_options.chessboard_rows,
                         chessboard_options.square_size, image_size, points3D[0], current_target_points_);

    double init_rms = std::numeric_limits<double>::max();
    bool in_init_phase = true;

    while (run_extraction_) {
      extract_.Wait();
      if (!run_extraction_) {
        // double check to prevent lengthy pose suggestion blocking the calibration
        return;
      }
      if (extractor_->Extract(currently_extracted_image_, *current_pixmap_) == FeatureExtractor::Status::SUCCESS) {
        ShowCapturePossible(true);
      }
      else {
        currently_extracted_image_.SetPoints2D({});
        ShowCapturePossible(false);
      }

      tracker.Update(currently_extracted_image_.Points2D());

      if (in_init_phase && !currently_extracted_image_.Points2D().empty()) {
        // during init phase calibrate on one image only to get valid focal length for pinhole camera
        // copy camera and only replace if the calibration rms is better than current camera
        colmap::Camera camera = calibration.Camera();
        Eigen::Quaterniond rot = currently_extracted_image_.Pose().rotation;
        Eigen::Vector3d trans = currently_extracted_image_.Pose().translation;
        std::vector<Eigen::Quaterniond*> rotations{&rot};
        std::vector<Eigen::Vector3d*> translations{&trans};
        double rms = opencv_calibration::CalibrateCamera(points3D, {currently_extracted_image_.Points2D()}, camera, true, true,
                                                         rotations, translations);
        // ignore calibrated principle point to prevent "jumping" of the target pose
        // TODO: this could be done via CV_CALIB_FIX_PRINCIPLE
        camera.SetPrincipalPointX(image_width / 2.0);
        camera.SetPrincipalPointY(image_height / 2.0);
        if (rms < init_rms) {
          // use best rms camera
          init_rms = rms;
          calibration.SetCamera(camera);
        }

        EstimateNextBestPose(calibration, chessboard_options.chessboard_columns, chessboard_options.chessboard_rows,
                             chessboard_options.square_size, image_size, points3D[0], current_target_points_);
        tracker.SetTargetPoints(current_target_points_);
      }

      if (tracker.TargetPointsReached() || (acquire_ && !currently_extracted_image_.Points2D().empty())) {
        Save(calibration, currently_extracted_image_, std::move(current_pixmap_));

        if (in_init_phase && calibration.Images().size() >= 3) {
          // transition out of init phase, now use target camera model
          in_init_phase = false;
          colmap::Camera& init_camera = calibration.Camera();
          colmap::Camera calib_camera = colmap::Camera::CreateFromModelName(
              colmap::kInvalidCameraId, CameraModel::CameraModels().at(dialog_options_.camera_model).model_name,
              init_camera.FocalLength(), init_camera.width, init_camera.height);

          calibration.SetCamera(calib_camera);
        }

        if (!in_init_phase) {
          calibrator.Calibrate(calibration);
          EstimateNextBestPose(calibration, chessboard_options.chessboard_columns, chessboard_options.chessboard_rows,
                               chessboard_options.square_size, image_size, points3D[0], current_target_points_);
          tracker.SetTargetPoints(current_target_points_);
        }
      }

      acquire_ = false;
      new_extraction_ = true;
    }
  }

  // Add the image to the calibration. If configured also save to file.
  void calibmar::LivestreamCalibrationRunner::Save(Calibration& calibration, Image image, std::unique_ptr<Pixmap> pixmap) {
    // Show shutter animation
    QMetaObject::invokeMethod(extraction_widget_,
                              [extraction_widget = extraction_widget_]() { extraction_widget->SignalImageAcquisition(); });

    // save a copy of the last image for the offset visualization
    last_pixmap_ = std::make_unique<Pixmap>(pixmap->Clone());

    if (std::holds_alternative<ArucoSiftFeatureExtractor::Options>(dialog_options_.calibration_target_options)) {
      ArucoSiftFeatureExtractor* extractor = static_cast<ArucoSiftFeatureExtractor*>(extractor_.get());

      // for 3D target the full sift feature detection is too slow to do it live, so it is only done for saving
      extractor->ExtractFull(image, *pixmap);
    }

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
    else {
      std::string img_number = std::to_string(image_counter_);
      // This is needed since for 3D calibration the colmap DB requires an existing image name
      image.SetName(img_number);
    }

    size_t id = calibration.AddImage(image);
    // for image names
    image_counter_++;

    // display image in sidebar
    std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
    if (dialog_options_.save_images_directory.has_value()) {
      data->image_name = image.Name();
    }

    data->image = std::move(pixmap);
    data->image_data = &calibration.Image(id);
    data->status = ExtractionImageWidget::Status::SUCCESS;
    QMetaObject::invokeMethod(extraction_widget_, [extraction_widget = extraction_widget_, data = std::move(data),
                                                   target_visualizer = target_visualizer_.get()]() mutable {
      extraction_widget->AddExtractionItem(std::move(data), *target_visualizer);
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