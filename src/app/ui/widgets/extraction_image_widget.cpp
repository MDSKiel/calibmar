#include "extraction_image_widget.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/widgets/image_widget.h"

namespace {
  QLabel* CreateImageNameLabel(std::string& image_name, int width) {
    // Assumes a file path
    std::size_t found = image_name.find_last_of("/\\");
    QString name =
        found == std::string::npos ? QString::fromStdString(image_name) : QString::fromStdString(image_name.substr(found + 1));

    QLabel* name_label = new QLabel();
    name = name_label->fontMetrics().elidedText(name, Qt::TextElideMode::ElideLeft, width - 20);
    name_label->setText(name);
    return name_label;
  }

  QWidget* CreateErrorWidget(const std::string& error, int widget_width) {
    QLabel* label = new QLabel(QString::fromStdString(error));
    label->setFixedSize(widget_width, widget_width * 0.7);
    label->setAlignment(Qt::AlignCenter);
    return label;
  }

  QWidget* CreateUndetectedWidget(calibmar::ExtractionImageWidget::Data& data, int widget_width) {
    cv::Mat& cornerMat = data.image->Data();

    if (cornerMat.channels() == 1) {
      // reallocate in place, this is supported by opencv, the old one gets destroyed if refrence count == 0
      cv::cvtColor(cornerMat, cornerMat, cv::COLOR_GRAY2RGB);
    }

    calibmar::Pixmap pixmap;
    pixmap.Assign(cornerMat);

    std::unique_ptr<calibmar::Pixmap> scaled = std::make_unique<calibmar::Pixmap>();
    double f = (double)widget_width / pixmap.Width();
    cv::resize(pixmap.Data(), scaled->Data(), cv::Size(), f, f);

    cv::rectangle(scaled->Data(), cv::Point(0, 0), cv::Point(scaled->Width(), scaled->Height()), cv::Scalar(0, 0, 255), 8);

    calibmar::ImageWidget* image_widget = new calibmar::ImageWidget();
    image_widget->SetImage(std::move(scaled));
    image_widget->adjustSize();

    // esp. needed for livestream sidebar layout
    image_widget->setSizePolicy(QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Minimum);

    return image_widget;
  }

  QWidget* CreateImageWidget(calibmar::ExtractionImageWidget::Data& data, const calibmar::TargetVisualizer& target_visualizer,
                             int widget_width) {
    cv::Mat& cornerMat = data.image->Data();

    if (cornerMat.channels() == 1) {
      // reallocate in place, this is supported by opencv, the old one gets destroyed if refrence count == 0
      cv::cvtColor(cornerMat, cornerMat, cv::COLOR_GRAY2RGB);
    }
    calibmar::Pixmap pixmap;
    pixmap.Assign(cornerMat);
    if (data.image_data) {
      target_visualizer.DrawTargetOnImage(pixmap, *data.image_data);
    }
    std::unique_ptr<calibmar::Pixmap> scaled = std::make_unique<calibmar::Pixmap>();
    double f = (double)widget_width / pixmap.Width();
    cv::resize(pixmap.Data(), scaled->Data(), cv::Size(), f, f);

    calibmar::ImageWidget* image_widget = new calibmar::ImageWidget();
    image_widget->SetImage(std::move(scaled));
    image_widget->adjustSize();

    // esp. needed for livestream sidebar layout
    image_widget->setSizePolicy(QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Minimum);

    return image_widget;
  }
}

namespace calibmar {

  ExtractionImageWidget::ExtractionImageWidget(std::unique_ptr<Data> data, const class TargetVisualizer& target_visualizer,
                                               QWidget* parent, std::optional<int> target_width)
      : QWidget(parent), target_visualizer_(target_visualizer), image_name_(data->image_name) {
    if (target_width.has_value()) {
      widget_width_ = target_width.value();
    }
    else {
      widget_width_ = calibmar::ExtractionImageWidget::GetDefaultWidth();
    }

    QWidget* content;
    switch (data->status) {
      case Status::SUCCESS:
        content = CreateImageWidget(*data, target_visualizer, widget_width_);
        break;
      case Status::DETECTION_ERROR:
        content = CreateUndetectedWidget(*data, widget_width_);
        break;
      case Status::READ_ERROR:
        content = CreateErrorWidget("Could not read image", widget_width_);
        break;
      case Status::IMAGE_DIMENSION_MISSMATCH:
        content = CreateErrorWidget("Image dimensions do not match first image", widget_width_);
        break;
      default:
        content = new QLabel();
    }

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(content);

    if (!data->image_name.empty()) {
      QWidget* name = CreateImageNameLabel(data->image_name, widget_width_);
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

  int ExtractionImageWidget::GetDefaultWidth() {
    return QGuiApplication::screens().first()->availableGeometry().width() * (1.0 / 8);
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
