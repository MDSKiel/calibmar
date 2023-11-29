#include "extraction_images_widget.h"


namespace calibmar {

  ExtractionImagesWidget::ExtractionImagesWidget(
      QWidget* parent, const std::function<void(const std::string&, const TargetVisualizer&)> double_click_callback)
      : QWidget(parent), double_click_callback_(double_click_callback) {
    main_layout_ = new FlowLayout(this);
    setContentsMargins(0, 0, 0, 0);
  }

  void ExtractionImagesWidget::AddImage(ExtractionImageWidget* widget) {
    main_layout_->addWidget(widget);
    update();
  }

  void ExtractionImagesWidget::mouseDoubleClickEvent(QMouseEvent* event) {
    ExtractionImageWidget* widget = nullptr;
    for (size_t i = 0; i < main_layout_->count(); i++) {
      QLayoutItem* item = main_layout_->itemAt(i);
      if (item->geometry().contains(event->pos())) {
        widget = static_cast<ExtractionImageWidget*>(item->widget());
        break;
      }
    }

    if (widget && double_click_callback_) {
      const TargetVisualizer& visualizer = widget->TargetVisualizer();
      const std::string& image_name = widget->ImageName();
      double_click_callback_(image_name, visualizer);
    }
  }
}