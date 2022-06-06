#pragma once

#include "calibmar/core/pixmap.h"

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class ImageWidget : public QWidget {
   public:
    ImageWidget(QWidget* parent = nullptr);

    void SetImage(std::unique_ptr<Pixmap> image);

    QSize drawSize() const;
    QSize sizeHint() const;

   protected:
    void paintEvent(QPaintEvent* event);
    void resizeEvent(QResizeEvent* event);

   private:
    std::unique_ptr<Pixmap> image_;
    QImage qimage_;
    QRect image_rect_;
    QSize size_hint_;

    void UpdateImageRect();
  };
}