#include "capture_button.h"

namespace calibmar {
  CaptureButton::CaptureButton(const QString& text, QWidget* parent) : QPushButton(text, parent) {}

  bool CaptureButton::eventFilter(QObject* obj, QEvent* event) {
    if (event->type() == QEvent::KeyRelease) {
      QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
      if (keyEvent->key() == Qt::Key_Space) {
        released();
        return true;
      }
    }

    // standard event processing
    return QObject::eventFilter(obj, event);
  }
}