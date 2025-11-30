#include "extraction_images_widget.h"
#include <deque>

namespace calibmar {

  ExtractionImagesWidget::ExtractionImagesWidget(
      QWidget* parent, const std::function<void(const std::string&, const TargetVisualizer&)> double_click_callback)
      : QWidget(parent), double_click_callback_(double_click_callback) {
    main_layout_ = new ImageFlowLayout(this);
    setContentsMargins(0, 0, 0, 0);
  }

  void ExtractionImagesWidget::AddExtractionItem(QWidget* widget, const std::string& key) {
    items_[key] = widget;
    main_layout_->addWidget(widget);
    update();
  }

  void ExtractionImagesWidget::SetOrder(const std::vector<std::string>& order) {
    if (order.size() != items_.size()) {
      return;
    }

    // Ensure all items are accounted for
    std::deque<std::pair<std::string, QWidget*>> items;
    for (const std::string& key : order) {
      const auto& item = items_.find(key);
      if (item != items_.end()) {
        items.push_back({item->first, item->second});
      }
    }
    if (items.size() != items_.size()) {
      return;
    }

    for (const auto& item : items_) {
      main_layout_->removeWidget(item.second);
    }
    for (const auto& item : items) {
      main_layout_->addWidget(item.second);
    }
    main_layout_->update();
  }

  void ExtractionImagesWidget::SetPerViewRMS(const std::unordered_map<std::string, double>& per_view_rms) {
    QList<ExtractionImageWidget*> imgs = this->findChildren<ExtractionImageWidget*>("ExtractionImageWidget");

    if (imgs.size() != per_view_rms.size()) {
      return;
    }
    for (ExtractionImageWidget* img : imgs) {
      const auto& rms = per_view_rms.find(img->ImageName());
      if (rms != per_view_rms.end()) {
        img->SetRMS(rms->second);
      }
    }
    main_layout_->update();
  }

  void ExtractionImagesWidget::mouseDoubleClickEvent(QMouseEvent* event) {
    if (event->button() != Qt::MouseButton::LeftButton) {
      return;
    }
    ExtractionImageWidget* widget = nullptr;
    QWidget* child = childAt(event->pos());
    if (child && child->parent()) {
      widget = dynamic_cast<ExtractionImageWidget*>(child->parent());
    }

    if (widget && double_click_callback_) {
      const TargetVisualizer& visualizer = widget->TargetVisualizer();
      const std::string& image_name = widget->ImageName();
      double_click_callback_(image_name, visualizer);
    }
  }

  void ExtractionImagesWidget::contextMenuEvent(QContextMenuEvent* event) {
    ExtractionImageWidget* widget = nullptr;
    QWidget* child = childAt(event->pos());
    if (child && child->parent()) {
      widget = dynamic_cast<ExtractionImageWidget*>(child->parent());
    }

    if (widget) {
      QMenu* menu = new QMenu(this);

      menu->addAction("Open Directory", this, [widget]() {
        QString directory = QFileInfo(QString::fromStdString(widget->ImageName())).absolutePath();
        // explicitly call xdg-open on linux, Qt does not seem to work on e.g. Ubuntu22.04
#ifdef __linux__
        std::stringstream ss;
        ss << "xdg-open " << directory.toStdString();
        std::system(ss.str().c_str());
#else
        QDesktopServices::openUrl(QUrl::fromLocalFile(directory));
#endif
      });

      menu->exec(event->globalPos());
    }
  }
}