#include "stereo_files_calibration_runner.h"
#include "calibmar/calibrators/calibrator3D.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/calibrators/stereo_calibrator.h"
#include "calibmar/extractors/aruco_sift_extractor.h"
#include "calibmar/extractors/sift_extractor.h"
#include "calibmar/readers/filesystem_reader.h"

#include "ui/widgets/calibration_result_widget.h"

#include <colmap/scene/reconstruction.h>

namespace {
  std::unique_ptr<calibmar::ExtractionImageWidget::Data> ReadAndExtractImage(calibmar::ImageReader& reader,
                                                                             calibmar::FeatureExtractor& extractor,
                                                                             calibmar::Image& image) {
    using namespace calibmar;

    std::unique_ptr<Pixmap> pixmap = std::make_unique<Pixmap>();
    ImageReader::Status reader_status = reader.Next(image, *pixmap);
    FeatureExtractor::Status extractor_status;
    std::unique_ptr<ExtractionImageWidget::Data> data = std::make_unique<ExtractionImageWidget::Data>();
    if (reader_status == ImageReader::Status::SUCCESS) {
      data->image_name = image.Name();

      extractor_status = extractor.Extract(image, *pixmap);

      data->image = std::move(pixmap);
      data->status = ExtractionImageWidget::ConvertStatus(extractor_status);
    }
    else {
      data->status = ExtractionImageWidget::ConvertStatus(reader_status);
    }

    return std::move(data);
  }
}

namespace calibmar {

  StereoFilesCalibrationRunner::StereoFilesCalibrationRunner(CalibrationWidget* calibration_widget,
                                                             StereoFileCalibrationDialog::Options options)
      : calibration_widget_(calibration_widget), options_(options) {}

  bool StereoFilesCalibrationRunner::Run(Calibration& calibration1, Calibration& calibration2) {
    FilesystemImageReader::Options reader_options1;
    reader_options1.image_directory = options_.images_directory1;
    FilesystemImageReader::Options reader_options2;
    reader_options2.image_directory = options_.images_directory2;
    FilesystemImageReader reader1(reader_options1);
    FilesystemImageReader reader2(reader_options2);

    try {
      // If the directories contain no images, requesting width/height can fail
      std::pair<int, int> image_size1{reader1.ImagesWidth(), reader1.ImagesHeight()};
      std::pair<int, int> image_size2{reader2.ImagesWidth(), reader2.ImagesHeight()};

      ChessboardFeatureExtractor::Options extractor_options;
      extractor_options.chessboard_columns = options_.calibration_target_options.chessboard_columns;
      extractor_options.chessboard_rows = options_.calibration_target_options.chessboard_rows;
      extractor_options.square_size = options_.calibration_target_options.square_size;
      ChessboardFeatureExtractor extractor(extractor_options);
      calibration1.SetPoints3D(extractor.Points3D());
      calibration1.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(options_.calibration_target_options));
      calibration2.SetPoints3D(extractor.Points3D());
      calibration2.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(options_.calibration_target_options));

      StereoCalibrator::Options calibrator_options;
      calibrator_options.estimate_pose_only = options_.estimate_pose_only;
      colmap::Camera camera1;
      colmap::Camera camera2;
      if (options_.initial_camera_parameters1.has_value()) {
        camera1 = CameraModel::InitCamera(options_.camera_model1, image_size1, options_.initial_camera_parameters1.value());
        calibrator_options.use_intrinsics_guess = true;
      }
      else {
        calibrator_options.camera_model1 = options_.camera_model1;
        calibrator_options.image_size1 = image_size1;
      }

      if (options_.initial_camera_parameters2.has_value()) {
        camera2 = CameraModel::InitCamera(options_.camera_model2, image_size2, options_.initial_camera_parameters2.value());
        calibrator_options.use_intrinsics_guess = true;
      }
      else {
        calibrator_options.camera_model2 = options_.camera_model2;
        calibrator_options.image_size2 = image_size1;
      }

      calibration1.SetCamera(camera1);
      calibration2.SetCamera(camera2);

      StereoCalibrator calibrator(calibrator_options);
      std::unique_ptr<TargetVisualizer> target_visualizer = std::make_unique<ChessboardTargetVisualizer>(
          options_.calibration_target_options.chessboard_columns, options_.calibration_target_options.chessboard_rows);

      calibration_widget_->SetTargetVisualizer(std::move(target_visualizer));

      while (reader1.HasNext() && reader2.HasNext()) {
        Image image1, image2;
        std::unique_ptr<calibmar::ExtractionImageWidget::Data> data1 = ReadAndExtractImage(reader1, extractor, image1);
        std::unique_ptr<calibmar::ExtractionImageWidget::Data> data2 = ReadAndExtractImage(reader2, extractor, image2);

        // currently ignore read errors. Check visualization, when reenabling.
        // files that can not be read are assumed to be unrelated, i.e. calibration files and are skipped
        while (data1->status == ExtractionImageWidget::Status::READ_ERROR && reader1.HasNext()) {
          data1 = ReadAndExtractImage(reader1, extractor, image1);
        }
        while (data2->status == ExtractionImageWidget::Status::READ_ERROR && reader2.HasNext()) {
          data2 = ReadAndExtractImage(reader2, extractor, image2);
        }
        if (data1->status == ExtractionImageWidget::Status::READ_ERROR ||
            data2->status == ExtractionImageWidget::Status::READ_ERROR) {
          continue;
        }

        if (data1->status == ExtractionImageWidget::Status::SUCCESS && data2->status == ExtractionImageWidget::Status::SUCCESS) {
          size_t id = calibration1.AddImage(image1);
          data1->image_data = calibration1.Image(id);
          id = calibration2.AddImage(image2);
          data2->image_data = calibration2.Image(id);
        }

        QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, data1 = std::move(data1),
                                                        data2 = std::move(data2)]() mutable {
          QFrame* frame = new QFrame();
          frame->setFrameStyle(QFrame::StyledPanel);
          QGridLayout* grid = new QGridLayout(frame);
          ExtractionImageWidget* image_widget1 =
              new ExtractionImageWidget(std::move(data1), calibration_widget->TargetVisualizer());
          ExtractionImageWidget* image_widget2 =
              new ExtractionImageWidget(std::move(data2), calibration_widget->TargetVisualizer());
          grid->addWidget(image_widget1, 0, 0);
          grid->addWidget(image_widget2, 0, 1);
          frame->setLayout(grid);

          calibration_widget->AddExtractionItem(frame);
        });
      }

      QMetaObject::invokeMethod(calibration_widget_,
                                [calibration_widget = calibration_widget_]() { calibration_widget->StartCalibration(); });

      calibrator.Calibrate(calibration1, calibration2);
    }
    catch (std::exception& ex) {
      std::string message(ex.what());

      QMetaObject::invokeMethod(calibration_widget_, [calibration_widget = calibration_widget_, message]() {
        calibration_widget->EndCalibration(new CalibrationResultWidget(message));
      });
      return false;
    }

    QMetaObject::invokeMethod(calibration_widget_,
                              [calibration_widget = calibration_widget_, &calibration1, &calibration2]() mutable {
      QWidget* frame = new QWidget();
      QVBoxLayout* layout = new QVBoxLayout(frame);
      QMargins margin = layout->contentsMargins();
      margin.setLeft(0);
      margin.setRight(0);
      layout->setContentsMargins(margin);
      layout->addWidget(new QLabel("<h2>Camera 1</h2>"));
      layout->addWidget(new CalibrationResultWidget(calibration1));
      layout->addWidget(new QLabel("<h2>Camera 2</h2>"));
      layout->addWidget(new CalibrationResultWidget(calibration2));

      calibration_widget->EndCalibration(frame);
    });

    return true;
  }
}
