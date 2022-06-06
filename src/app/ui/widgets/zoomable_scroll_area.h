#pragma once

#include <QtWidgets>

namespace calibmar {
  class ZoomableScrollArea : public QScrollArea {
   public:
    ZoomableScrollArea(QWidget* parent = nullptr);

   protected:
    void wheelEvent(QWheelEvent* event);
  };
}