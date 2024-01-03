#include "collapsible_widget.h"

#include <QFrame>
#include <QScrollArea>
#include <QToolButton>

namespace calibmar {
  CollapsibleWidget::CollapsibleWidget(const QString& title, const std::function<void(bool)> collapse_toggeled_callback,
                                       QWidget* parent)
      : collapse_toggeled_callback_(collapse_toggeled_callback) {
    QToolButton* toggle_button = new QToolButton(this);
    toggle_button->setStyleSheet("QToolButton { border: none; }");
    toggle_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toggle_button->setArrowType(Qt::ArrowType::RightArrow);
    toggle_button->setText(title);
    toggle_button->setCheckable(true);
    toggle_button->setChecked(false);
    toggle_button->setCursor(QCursor(Qt::PointingHandCursor));

    QFrame* header_line = new QFrame(this);
    header_line->setFrameShape(QFrame::HLine);
    header_line->setFrameShadow(QFrame::Sunken);
    header_line->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);

    main_layout_ = new QGridLayout(this);
    vertical_spacing_ = main_layout_->verticalSpacing();
    main_layout_->setVerticalSpacing(0);
    main_layout_->addWidget(toggle_button, 0, 0, 1, 1, Qt::AlignLeft);
    main_layout_->addWidget(header_line, 0, 2, 1, 1);

    QObject::connect(toggle_button, &QToolButton::clicked, [toggle_button, this](const bool checked) {
      toggle_button->setArrowType(checked ? Qt::ArrowType::DownArrow : Qt::ArrowType::RightArrow);
      if (this->widget_) {
        widget_->setFixedHeight(checked ? target_height_ : 0);
        main_layout_->setVerticalSpacing(checked ? vertical_spacing_ : 0);
      }

      if(collapse_toggeled_callback_){
        collapse_toggeled_callback_(checked);
      }
    });
  }

  QWidget* CollapsibleWidget::SetWidget(QWidget* widget, int target_height) {
    QWidget* old = nullptr;
    target_height_ = target_height;
    widget->setFixedHeight(0);
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    if (widget_) {
      main_layout_->replaceWidget(widget_, widget);
      old = widget_;
    }
    else {
      main_layout_->addWidget(widget, 1, 0, 1, 3);
    }
    widget_ = widget;
    return old;
  }

  QWidget* CollapsibleWidget::TakeWidget() {
    if (widget_) {
      main_layout_->removeWidget(widget_);
    }
    QWidget* old = widget_;
    widget_ = nullptr;
    return old;
  }
}