#pragma once

#include <QGridLayout>
#include <QWidget>
#include <QtCore>

namespace calibmar {
  class CollapsibleWidget : public QWidget {
   public:
    CollapsibleWidget(const QString& title = "", const std::function<void(bool)> collapse_toggeled_callback = nullptr,
                      QWidget* parent = nullptr);

    // Set Widget. If any exists it will be replaced and the existing returned.
    QWidget* SetWidget(QWidget* widget, int target_height);
    QWidget* TakeWidget();

   private:
    int target_height_ = 0;
    int vertical_spacing_ = 0;
    QWidget* widget_ = nullptr;
    QGridLayout* main_layout_;
    const std::function<void(bool)> collapse_toggeled_callback_;
  };
}