#include "extraction_image_widget.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace {
  int widget_width = 300;
  int widget_height = 300;

  QLabel* CreateImageNameLabel(std::string& image_name, int width) {
    // Assumes a file path
    std::size_t found = image_name.find_last_of("/\\");
    QString name =
        found == std::string::npos ? QString::fromStdString(image_name) : QString::fromStdString(image_name.substr(found + 1));

    QLabel* name_label = new QLabel();
    name = name_label->fontMetrics().elidedText(name, Qt::TextElideMode::ElideLeft, width);
    name_label->setText(name);
    return name_label;
  }

  QWidget* CreateErrorWidget(const std::string& error) {
    QLabel* label = new QLabel(QString::fromStdString(error));
    label->setFixedSize(widget_width * 0.7, widget_height * 0.7);
    label->setAlignment(Qt::AlignCenter);
    return label;
  }

  QWidget* CreateUndetectedWidget(calibmar::ExtractionImageWidget::Data& data) {
    cv::Mat& cornerMat = data.image->Data();

    if (cornerMat.channels() == 1) {
      // reallocate in place, this is supported by opencv, the old one gets destroyed if refrence count == 0
      cv::cvtColor(cornerMat, cornerMat, cv::COLOR_GRAY2RGB);
    }

    QImage image = QImage(cornerMat.data, cornerMat.cols, cornerMat.rows, cornerMat.step, QImage::Format_BGR888);
    QImage scaled = image.scaled(widget_width, widget_height, Qt::AspectRatioMode::KeepAspectRatio);
    QPainter painter(&scaled);
    painter.setPen(QPen(Qt::red, 4));
    painter.drawRect(2, 2, scaled.width() - 4, scaled.height() - 4);

    QLabel* image_label = new QLabel();
    image_label->setPixmap(QPixmap::fromImage(scaled));
    image_label->adjustSize();

    return image_label;
  }

  QWidget* CreateImageWidget(calibmar::ExtractionImageWidget::Data& data, const calibmar::TargetVisualizer& target_visualizer) {
    cv::Mat& cornerMat = data.image->Data();

    if (cornerMat.channels() == 1) {
      // reallocate in place, this is supported by opencv, the old one gets destroyed if refrence count == 0
      cv::cvtColor(cornerMat, cornerMat, cv::COLOR_GRAY2RGB);
    }
    calibmar::Pixmap pixmap;
    pixmap.Assign(cornerMat);
    target_visualizer.DrawTargetOnImage(pixmap, *data.image_data);
    QImage image = QImage(cornerMat.data, cornerMat.cols, cornerMat.rows, cornerMat.step, QImage::Format_BGR888);

    QImage scaled = image.scaled(widget_width, widget_height, Qt::AspectRatioMode::KeepAspectRatio);

    QLabel* image_label = new QLabel();
    image_label->setPixmap(QPixmap::fromImage(scaled));
    image_label->adjustSize();

    return image_label;
  }
}

namespace calibmar {

  ExtractionImageWidget::ExtractionImageWidget(std::unique_ptr<Data> data, const class TargetVisualizer& target_visualizer,
                                               QWidget* parent)
      : QWidget(parent), target_visualizer_(target_visualizer), image_name_(data->image_name) {
    widget_width = QGuiApplication::screens().first()->availableGeometry().width() * (1.0 / 8);
    widget_height = QGuiApplication::screens().first()->availableGeometry().height() * (1.0 / 8);

    QWidget* content;
    switch (data->status) {
      case Status::SUCCESS:
        content = CreateImageWidget(*(data.get()), target_visualizer);
        break;
      case Status::DETECTION_ERROR:
        content = CreateUndetectedWidget(*(data.get()));
        break;
      case Status::READ_ERROR:
        content = CreateErrorWidget("Could not read image");
        break;
      case Status::IMAGE_DIMENSION_MISSMATCH:
        content = CreateErrorWidget("Image dimensions do not match first image");
        break;
      default:
        content = new QLabel();
    }

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(content);

    if (!data->image_name.empty()) {
      QWidget* name = CreateImageNameLabel(data->image_name, content->width());
      layout->addWidget(name);
      layout->setAlignment(name, Qt::AlignHCenter | Qt::AlignBottom);
    }
  }

  const TargetVisualizer& ExtractionImageWidget::TargetVisualizer() {
    return target_visualizer_;
  }

  const std::string& ExtractionImageWidget::ImageName() {
    return image_name_;
  }

  ExtractionImageWidget::Status ExtractionImageWidget::ConvertStatus(FeatureExtractor::Status status) {
    switch (status) {
      case calibmar::FeatureExtractor::Status::SUCCESS:
        return Status::SUCCESS;
      case calibmar::FeatureExtractor::Status::DETECTION_ERROR:
        return Status::DETECTION_ERROR;
      default:
        throw new std::runtime_error("unkown status!");
    }
  }

  ExtractionImageWidget::Status ExtractionImageWidget::ConvertStatus(ImageReader::Status status) {
    switch (status) {
      case calibmar::ImageReader::Status::READ_ERROR:
        return Status::READ_ERROR;
      case calibmar::ImageReader::Status::IMAGE_DIMENSION_MISSMATCH:
        return Status::IMAGE_DIMENSION_MISSMATCH;
      case calibmar::ImageReader::Status::NO_MORE_IMAGES:
        return Status::READ_ERROR;
      case calibmar::ImageReader::Status::SUCCESS:
        return Status::SUCCESS;
      default:
        throw new std::runtime_error("unkown status!");
    }
  }
}
