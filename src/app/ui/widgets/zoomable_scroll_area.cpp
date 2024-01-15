#include "zoomable_scroll_area.h"

namespace calibmar {
  ZoomableScrollArea::ZoomableScrollArea(QWidget* parent) : QScrollArea(parent) {
    setAlignment(Qt::Alignment(Qt::AlignmentFlag::AlignCenter));
  }

  void ZoomableScrollArea::wheelEvent(QWheelEvent* event) {
    if (!event->modifiers().testFlag(Qt::KeyboardModifier::ControlModifier) || event->angleDelta().isNull()) {
      QScrollArea::wheelEvent(event);
      return;
    }
    event->accept();
    double inc = 0.1;
    // resize the contained widget relatively depending on wheel direction
    double scale = event->angleDelta().y() > 0 ? 1 + inc : 1 - inc;

    // limit scaling
    QSize new_size = widget()->size() * scale;
    int limited = std::clamp(std::max(new_size.width(), new_size.height()), 100, 50000);
    new_size = widget()->size().scaled(limited, limited, Qt::KeepAspectRatio);

    // due to limiting the actual scale might be smaller
    scale = static_cast<double>(new_size.width()) / widget()->width();
    QPointF scrollbar_pos = QPointF(horizontalScrollBar()->value(), verticalScrollBar()->value());
    QPointF delta_to_pos = event->posF() - widget()->pos();
    QPointF delta = delta_to_pos * scale - delta_to_pos;

    widget()->resize(new_size);

    // scroll to cursor
    horizontalScrollBar()->setValue(scrollbar_pos.x() + delta.x());
    verticalScrollBar()->setValue(scrollbar_pos.y() + delta.y());
  }
}