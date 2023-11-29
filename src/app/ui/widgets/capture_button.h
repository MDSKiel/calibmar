#pragma once

#include <QtWidgets>

namespace calibmar {

  class CaptureButton : public QPushButton {
   public:
    explicit CaptureButton(const QString& text, QWidget* parent = nullptr);

   protected:
    bool eventFilter(QObject* obj, QEvent* event) override;
  };
}