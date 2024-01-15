#include "image_widget.h"
#include "ui/utils/render.h"

namespace calibmar {

  ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent), size_hint_(QSize()), image_(nullptr) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  void ImageWidget::SetImage(std::unique_ptr<Pixmap> image) {
    image_ = std::move(image);
    qimage_ = render::GetQImageFromPixmap(*image_);
    QSize size(image_->Width(), image_->Height());
    if (size_hint_ != size) {
      size_hint_ = size;
      updateGeometry();
    }
    UpdateImageRect();
    update();
  }

  std::unique_ptr<Pixmap> ImageWidget::TakeImage() {
    qimage_ = QImage(); // invalidate the qimage referencing the pixmap data
    size_hint_ = QSize();
    return std::move(image_);
  }

  QSize ImageWidget::drawSize() const {
    return image_rect_.size();
  }

  QSize ImageWidget::sizeHint() const {
    return size_hint_;
  }

  void ImageWidget::paintEvent(QPaintEvent* event) {
    if (!image_) {
      return;
    }

    QPainter painter(this);
    // If the image is shrunk do so smoothely (for zoom nearest nieghbor is desired)
    if (image_rect_.width() < qimage_.width()) {
      painter.setRenderHint(QPainter::SmoothPixmapTransform);
    }
    painter.drawImage(image_rect_, qimage_);
  }

  void ImageWidget::resizeEvent(QResizeEvent* event) {
    UpdateImageRect();
  }

  void ImageWidget::UpdateImageRect() {
    if (!image_) {
      return;
    }

    // Draw image in the center of available space preserving the aspect ratio
    QSize target_size = QSize(image_->Width(), image_->Height()).scaled(size(), Qt::AspectRatioMode::KeepAspectRatio);
    QSize diff = size() - target_size;
    image_rect_ = QRect(diff.width() / 2, diff.height() / 2, target_size.width(), target_size.height());
  }
}