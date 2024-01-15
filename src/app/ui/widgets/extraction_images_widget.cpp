#include "extraction_images_widget.h"

namespace calibmar {

  ExtractionImagesWidget::ExtractionImagesWidget(
      QWidget* parent, const std::function<void(const std::string&, const TargetVisualizer&)> double_click_callback)
      : QWidget(parent), double_click_callback_(double_click_callback) {
    main_layout_ = new ImageFlowLayout(this);
    setContentsMargins(0, 0, 0, 0);
  }

  void ExtractionImagesWidget::AddImage(QWidget* widget) {
    main_layout_->addWidget(widget);
    update();
  }

  void ExtractionImagesWidget::mouseDoubleClickEvent(QMouseEvent* event) {    
    ExtractionImageWidget* widget = dynamic_cast<ExtractionImageWidget*>(childAt(event->pos())->parent());

    if (widget && double_click_callback_) {
      const TargetVisualizer& visualizer = widget->TargetVisualizer();
      const std::string& image_name = widget->ImageName();
      double_click_callback_(image_name, visualizer);
    }
  }
}