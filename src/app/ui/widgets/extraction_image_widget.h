#pragma once

#include "calibmar/core/pixmap.h"
#include "calibmar/extractors/extractor.h"
#include "calibmar/readers/image_reader.h"
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
      std::vector<Eigen::Vector2d> points;
      std::string image_name;
      int chessboard_columns;
      int chessboard_rows;
    };

    ExtractionImageWidget(std::unique_ptr<Data> data, QWidget* parent = nullptr);

    std::pair<int, int> ColumnsRows();
    const std::string& ImageName();

    static Status ConvertStatus(FeatureExtractor::Status status);
    static Status ConvertStatus(ImageReader::Status status);

   private:
    std::pair<int, int> cols_rows_;
    std::string image_name_;
  };
}