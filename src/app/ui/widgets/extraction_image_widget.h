#pragma once

#include "calibmar/core/pixmap.h"
#include "calibmar/extractors/extractor.h"
#include "calibmar/readers/image_reader.h"
#include "ui/utils/calibration_target_visualization.h"
#include <Eigen/Core>
#include <QtCore>
#include <QtWidgets>

namespace calibmar {
  // Wiget that displays an extracted image
  class ExtractionImageWidget : public QWidget {
   public:
    enum class Status { SUCCESS, DETECTION_ERROR, READ_ERROR, IMAGE_DIMENSION_MISSMATCH };
    struct Data {
      Status status;
      std::unique_ptr<Pixmap> image;
      Image* image_data;
      std::string image_name; // image name is kept separately in case extraction fails
    };

    ExtractionImageWidget(std::unique_ptr<Data> data, const class TargetVisualizer& target_visualizer, QWidget* parent = nullptr);

    const class TargetVisualizer& TargetVisualizer();
    const std::string& ImageName();

    static Status ConvertStatus(FeatureExtractor::Status status);
    static Status ConvertStatus(ImageReader::Status status);

   private:
    std::string image_name_;
    const class TargetVisualizer& target_visualizer_;
  };
}