#include "livestream_extraction_widget.h"
#include "extraction_image_widget.h"
#include "ui/utils/heatmap.h"
#include <opencv2/imgproc.hpp>

namespace calibmar {
  LiveStreamExtractionWidget::LiveStreamExtractionWidget(QWidget* parent) : QWidget(parent) {
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    QHBoxLayout* top_layout = new QHBoxLayout(this);
    QVBoxLayout* side_layout = new QVBoxLayout(this);

    image_widget_ = new ImageWidget(this);

    // 'missuse' a scroll area as frame
    QScrollArea* image_frame = new QScrollArea(this);
    image_frame->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    image_frame->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    image_frame->setWidgetResizable(true);
    image_frame->setWidget(image_widget_);

    live_layout_ = new QVBoxLayout();
    live_layout_->setAlignment(Qt::AlignTop);
    live_layout_->addWidget(image_frame);

    extraction_images_scroll_area_ = new QScrollArea(this);
    QWidget* extraction_images_side_bar = new QWidget(extraction_images_scroll_area_);
    extraction_images_layout_ = new QVBoxLayout(extraction_images_side_bar);
    extraction_images_layout_->setAlignment(Qt::AlignTop);
    extraction_images_scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    extraction_images_scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    extraction_images_scroll_area_->setWidgetResizable(true);
    extraction_images_scroll_area_->setWidget(extraction_images_side_bar);
    int top, right, bottom, left;
    extraction_images_layout_->getContentsMargins(&top, &right, &bottom, &left);
    extraction_images_scroll_area_->setFixedWidth(ExtractionImageWidget::GetDefaultWidth() + left + right);

    heatmap_widget_ = new ImageWidget(this);
    heatmap_widget_->setVisible(false);

    connect(extraction_images_scroll_area_->verticalScrollBar(), &QScrollBar::rangeChanged, this,
            [this](int min, int max) { this->extraction_images_scroll_area_->verticalScrollBar()->setValue(max); });

    done_button_ = new QPushButton("Calibrate", this);

    side_layout->addWidget(heatmap_widget_);
    side_layout->addWidget(extraction_images_scroll_area_);
    top_layout->addLayout(live_layout_);
    top_layout->addLayout(side_layout);
    main_layout->addLayout(top_layout);
    main_layout->addWidget(done_button_);
    main_layout->setAlignment(done_button_, Qt::AlignRight);

    // flash animation
    flash_widget_ = new QWidget(this);
    flash_widget_->setStyleSheet("background-color: black");
    flash_widget_->setVisible(false);
    flash_widget_->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint | Qt::WindowSystemMenuHint);

    flash_animation_ = new QPropertyAnimation(flash_widget_, "windowOpacity", this);
    flash_animation_->setStartValue(1);
    flash_animation_->setEndValue(0);
    flash_animation_->setDuration(150);
    flash_animation_->setEasingCurve(QEasingCurve::OutQuad);
    connect(flash_animation_, &QPropertyAnimation::finished, [this]() { flash_widget_->hide(); });
  }

  void LiveStreamExtractionWidget::SetLiveStreamImage(std::unique_ptr<Pixmap> image) {
    image_widget_->SetImage(std::move(image));
    image_widget_->update();
  }

  void LiveStreamExtractionWidget::AddExtractionItem(std::unique_ptr<ExtractionImageWidget::Data> data,
                                                     const TargetVisualizer& target_visualizer) {
    // Update heatmap
    int width = data->image->Width();
    int height = data->image->Height();
    if (!heatmap_) {
      heatmap_ = std::make_unique<Pixmap>();
      heatmap_->Data() = cv::Mat::zeros(height, width, CV_8UC1);
    }

    heatmap::AddPointsToRawHeatmap(data->image_data->Points2D(), *heatmap_);
    std::unique_ptr<Pixmap> display_heatmap = image_widget_->TakeImage();
    heatmap::ApplyColorMapToRawHeatmap(*heatmap_, *display_heatmap);
    heatmap_widget_->SetImage(std::move(display_heatmap));

    heatmap_widget_->setFixedSize(
        QSize(width, height).scaled(extraction_images_scroll_area_->size(), Qt::AspectRatioMode::KeepAspectRatio));
    heatmap_widget_->setVisible(true);

    // Image
    int top, right, bottom, left;
    extraction_images_layout_->getContentsMargins(&top, &right, &bottom, &left);
    ExtractionImageWidget* extraction_image = new ExtractionImageWidget(std::move(data), target_visualizer, this);
    extraction_images_layout_->addWidget(extraction_image);
  }

  void LiveStreamExtractionWidget::AddLiveModeWidget(QWidget* widget) {
    live_layout_->addWidget(widget);
  }

  void LiveStreamExtractionWidget::SetCompleteButtonCallback(std::function<void()>& finish_callback) {
    connect(done_button_, &QPushButton::released, finish_callback);
  }

  void LiveStreamExtractionWidget::SignalImageAcquisition() {
    QSize image_size = image_widget_->drawSize();
    QPoint image_origin = image_widget_->mapToGlobal(image_widget_->pos());
    QSize offset = (image_widget_->size() - image_size) / 2;
    image_origin += {offset.width(), offset.height()};

    flash_widget_->resize(image_size);
    flash_widget_->move(image_origin);

    flash_widget_->show();
    flash_animation_->start();
  }

  std::vector<ExtractionImageWidget*> LiveStreamExtractionWidget::RemoveExtractionImagesWidgets() {
    std::vector<ExtractionImageWidget*> widgets;

    int num_widgets = extraction_images_layout_->count();
    for (int i = 0; i < num_widgets; i++) {
      ExtractionImageWidget* widget = static_cast<ExtractionImageWidget*>(extraction_images_layout_->itemAt(0)->widget());
      extraction_images_layout_->removeWidget(widget);
      widgets.push_back(widget);
    }

    return widgets;
  }
}