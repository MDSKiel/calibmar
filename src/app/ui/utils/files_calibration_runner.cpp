#include "files_calibration_runner.h"
#include "calibmar/calibrators/calibrator.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/readers/filesystem_reader.h"
#include "ui/widgets/calibration_result_widget.h"

namespace calibmar {

  FilesCalibrationRunner::FilesCalibrationRunner(CalibrationWidget* calibration_widget, FileCalibrationDialog::Options options)
      : calibration_widget_(calibration_widget), options_(options) {}

  bool FilesCalibrationRunner::Run(Calibration& calibration) {
    FilesystemImageReader::Options reader_options;
    reader_options.image_directory = options_.images_directory;
    FilesystemImageReader reader(reader_options);
    ChessboardFeatureExtractor::Options extractor_options;
    extractor_options.chessboard_columns = options_.chessboard_columns;
    extractor_options.chessboard_rows = options_.chessboard_rows;
    extractor_options.square_size = options_.square_size;
    ChessboardFeatureExtractor extractor(extractor_options);
    calibration.SetPoints3D(extractor.Points3D());
    calibration.SetCalibrationTargetInfo("chessboard, " + std::to_string(options_.chessboard_columns) + ", " +
                                         std::to_string(options_.chessboard_rows) + ", " + std::to_string(options_.square_size));

    try {
      while (reader.HasNext()) {
        Image image;
        std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
        ImageReader::Status reader_status = reader.Next(image, *pixmap);
        FeatureExtractor::Status extractor_status;
        std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
        data->chessboard_columns = options_.chessboard_columns;
        data->chessboard_rows = options_.chessboard_rows;
        if (reader_status == ImageReader::Status::SUCCESS) {
          data->image_name = image.Name();

          extractor_status = extractor.Extract(image, *pixmap);

          if (extractor_status == FeatureExtractor::Status::SUCCESS) {
            calibration.AddImage(image);
            data->points = image.Points2D();
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
          QMetaObject::invokeMethod(calibration_widget_, [this, data = std::move(data)]() mutable {
            calibration_widget_->AddExtractionItem(new ExtractionImageWidget(std::move(data)));
          });
        }
      }

      QMetaObject::invokeMethod(calibration_widget_,
                                [calibration_widget = calibration_widget_]() { calibration_widget->StartCalibration(); });

      if (options_.housing_calibration.has_value()) {
        HousingCalibrator::Options calibrator_options;
        calibrator_options.camera_model = options_.camera_model;
        calibrator_options.image_size = {reader.ImagesWidth(), reader.ImagesHeight()};
        calibrator_options.estimate_initial_dome_offset = options_.housing_calibration.value().first == HousingInterfaceType::DoubleLayerSphericalRefractive;
        calibrator_options.housing_interface = options_.housing_calibration.value().first;
        calibrator_options.camera_params = options_.initial_camera_parameters.value();
        calibrator_options.initial_housing_params = options_.housing_calibration.value().second;
        calibrator_options.pattern_cols_rows = {options_.chessboard_columns, options_.chessboard_rows};

        HousingCalibrator calibrator(calibrator_options);
        calibrator.Calibrate(calibration);
      }
      else {
        Calibrator::Options calibrator_options;
        if (options_.initial_camera_parameters.has_value()) {
          calibration.SetCamera(CameraModel::InitCamera(options_.camera_model, {reader.ImagesWidth(), reader.ImagesHeight()},
                                                        options_.initial_camera_parameters.value()));
          calibrator_options.use_intrinsics_guess = true;
        }
        else {
          calibrator_options.camera_model = options_.camera_model;
          calibrator_options.image_size = {reader.ImagesWidth(), reader.ImagesHeight()};
        }
        Calibrator calibrator(calibrator_options);
        calibrator.Calibrate(calibration);
      }
    }
    catch (std::exception& ex) {
      std::string message(ex.what());

      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget(message));
      });
      return false;
    }

    QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_,
                                                    last_pixmap = std::move(last_pixmap_), &calibration]() mutable {
      calibration_widget->EndCalibration(new CalibrationResultWidget(calibration, std::move(last_pixmap)));
    });

    return true;
  }
}
