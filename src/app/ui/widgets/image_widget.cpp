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
    painter.drawImage(image_rect_, qimage_);
  }

  void ImageWidget::resizeEvent(QResizeEvent* event) {
    UpdateImageRect();
  }

  void ImageWidget::UpdateImageRect() {
    if (!image_) {
      return;
    }

    int target_width = width();
    int target_height = height();

    float aspect_s = static_cast<float>(image_->Width()) / image_->Height();
    float aspect_t = static_cast<float>(target_width) / target_height;

    if (aspect_s < aspect_t) {
      image_rect_.setHeight(target_height);
      image_rect_.setWidth(aspect_s * image_rect_.height());
    }
    else {
      image_rect_.setWidth(target_width);
      image_rect_.setHeight(target_width / aspect_s);
    }
  }
}