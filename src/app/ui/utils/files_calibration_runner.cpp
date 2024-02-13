#include "files_calibration_runner.h"
#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/calibrator3D.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/sift_extractor.h"
#include "calibmar/readers/filesystem_reader.h"

#include "ui/widgets/calibration_result_widget.h"

#include <colmap/scene/reconstruction.h>

namespace {
  void SetupChessboardCalibration(const calibmar::ChessboardFeatureExtractor::Options& chessboard_options,
                                  const calibmar::FileCalibrationDialog::Options& options, calibmar::Calibration& calibration,
                                  std::pair<int, int> image_size, std::unique_ptr<calibmar::FeatureExtractor>& extractor,
                                  std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer,
                                  std::unique_ptr<calibmar::Calibrator>& calibrator) {
    using namespace calibmar;
    ChessboardFeatureExtractor::Options extractor_options;
    extractor_options.chessboard_columns = chessboard_options.chessboard_columns;
    extractor_options.chessboard_rows = chessboard_options.chessboard_rows;
    extractor_options.square_size = chessboard_options.square_size;
    std::unique_ptr<ChessboardFeatureExtractor> chessboard_extractor =
        std::make_unique<ChessboardFeatureExtractor>(extractor_options);
    calibration.SetPoints3D(chessboard_extractor->Points3D());
    calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(chessboard_options));

    if (options.housing_calibration.has_value()) {
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
  }

  void Setup3DTargetCalibration(calibmar::Target3DTargetOptionsWidget::Options target3d_options,
                                const calibmar::FileCalibrationDialog::Options& options, calibmar::Calibration& calibration,
                                std::pair<int, int> image_size, std::unique_ptr<calibmar::FeatureExtractor>& extractor,
                                std::unique_ptr<calibmar::TargetVisualizer>& target_visualizer,
                                std::unique_ptr<calibmar::Calibrator>& calibrator) {
    using namespace calibmar;

    calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(target3d_options));
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

    if (options.housing_calibration.has_value()) {
      calibration.Camera().refrac_model_id = colmap::CameraRefracModelNameToId(
          calibmar::HousingInterface::HousingInterfaces().at(options.housing_calibration.value().first).model_name);
      calibration.Camera().refrac_params = options.housing_calibration.value().second;
      calibrator_options.enable_refraction = true;
    }

    double aruco_mask_factor = 1;
    if (use_aruco) {
      ArucoSiftFeatureExtractor::Options options = std::get<ArucoSiftFeatureExtractor::Options>(target3d_options);
      aruco_mask_factor = options.masking_scale_factor;
      extractor = std::make_unique<ArucoSiftFeatureExtractor>(options);
    }
    else {
      SiftFeatureExtractor::Options options = std::get<SiftFeatureExtractor::Options>(target3d_options);
      extractor = std::make_unique<SiftFeatureExtractor>(options);
    }

    calibrator = std::make_unique<Calibrator3D>(calibrator_options);
    target_visualizer = std::make_unique<Target3DTargetVisualizer>(use_aruco, aruco_mask_factor);
  }
}

namespace calibmar {

  FilesCalibrationRunner::FilesCalibrationRunner(CalibrationWidget* calibration_widget, FileCalibrationDialog::Options options)
      : calibration_widget_(calibration_widget), options_(options) {}

  bool FilesCalibrationRunner::Run(Calibration& calibration) {
    FilesystemImageReader::Options reader_options;
    reader_options.image_directory = options_.images_directory;
    FilesystemImageReader reader(reader_options);

    std::pair<int, int> image_size;
    try {
      image_size = {reader.ImagesWidth(), reader.ImagesHeight()};
    }
    catch (std::exception& ex) {
      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget("No readable images in provided directory!"));
      });
      return false;
    }

    std::unique_ptr<FeatureExtractor> extractor;
    std::unique_ptr<TargetVisualizer> target_visualizer;
    std::unique_ptr<Calibrator> calibrator;

    bool is_3D_calibration = !std::holds_alternative<ChessboardFeatureExtractor::Options>(options_.calibration_target_options);

    Target3DTargetOptionsWidget::Options target3d_options;
    if (is_3D_calibration) {
      if (std::holds_alternative<ArucoSiftFeatureExtractor::Options>(options_.calibration_target_options)) {
        target3d_options = std::get<ArucoSiftFeatureExtractor::Options>(options_.calibration_target_options);
      }
      else {
        target3d_options = std::get<SiftFeatureExtractor::Options>(options_.calibration_target_options);
      }

      Setup3DTargetCalibration(target3d_options, options_, calibration, image_size, extractor, target_visualizer, calibrator);
    }
    else {
      ChessboardFeatureExtractor::Options chessboard_options =
          std::get<ChessboardFeatureExtractor::Options>(options_.calibration_target_options);

      SetupChessboardCalibration(chessboard_options, options_, calibration, image_size, extractor, target_visualizer, calibrator);
    }

    calibration_widget_->SetTargetVisualizer(std::move(target_visualizer));

    try {
      while (reader.HasNext()) {
        Image image;
        std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
        ImageReader::Status reader_status = reader.Next(image, *pixmap);
        FeatureExtractor::Status extractor_status;
        std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
        if (reader_status == ImageReader::Status::SUCCESS) {
          data->image_name = image.Name();

          extractor_status = extractor->Extract(image, *pixmap);

          if (extractor_status == FeatureExtractor::Status::SUCCESS) {
            size_t id = calibration.AddImage(image);
            data->image_data = &calibration.Image(id);
          }

          // save a copy of the last image for the offset visualization
          last_pixmap_ = std::make_unique<Pixmap>(pixmap->Clone());
          data->image = std::move(pixmap);
          data->status = ExtractionImageWidget::ConvertStatus(extractor_status);
        }
        else {
          data->status = ExtractionImageWidget::ConvertStatus(reader_status);
        }

        // currently ignore read errors. Check visualization, when reenabling.
        if (data->status == ExtractionImageWidget::Status::SUCCESS ||
            data->status == ExtractionImageWidget::Status::DETECTION_ERROR ||
            data->status == ExtractionImageWidget::Status::IMAGE_DIMENSION_MISSMATCH) {
          QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, data = std::move(data)]() mutable {
            calibration_widget->AddExtractionItem(
                new ExtractionImageWidget(std::move(data), calibration_widget->TargetVisualizer()));
          });
        }
      }

      QMetaObject::invokeMethod(calibration_widget_,
                                [calibration_widget = calibration_widget_]() { calibration_widget->StartCalibration(); });

      calibrator->Calibrate(calibration);
    }
    catch (std::exception& ex) {
      std::string message(ex.what());

      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget(message));
      });
      return false;
    }

    std::shared_ptr<colmap::Reconstruction> reconstruction;
    if (is_3D_calibration) {
      reconstruction = static_cast<Calibrator3D*>(calibrator.get())->Reconstruction();
    }

    QMetaObject::invokeMethod(calibration_widget_,
                              [calibration_widget = calibration_widget_, last_pixmap = std::move(last_pixmap_), &calibration,
                               reconstruction]() mutable {
      calibration_widget->EndCalibration(new CalibrationResultWidget(calibration, std::move(last_pixmap), reconstruction));
    });

    return true;
  }
}
