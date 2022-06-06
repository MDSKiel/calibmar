#include "livestream_extraction_widget.h"
#include "extraction_image_widget.h"

namespace calibmar {
  LiveStreamExtractionWidget::LiveStreamExtractionWidget(QWidget* parent) : QWidget(parent) {
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    QHBoxLayout* top_layout = new QHBoxLayout(this);

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

    side_scroll_area_ = new QScrollArea(this);
    QWidget* side_bar = new QWidget(side_scroll_area_);
    side_layout_ = new QVBoxLayout(side_bar);
    side_layout_->setAlignment(Qt::AlignTop);
    side_scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    side_scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    side_scroll_area_->setWidgetResizable(true);
    side_scroll_area_->setWidget(side_bar);
    side_scroll_area_->setFixedWidth(330);

    connect(side_scroll_area_->verticalScrollBar(), &QScrollBar::rangeChanged, this,
            [this](int min, int max) { this->side_scroll_area_->verticalScrollBar()->setValue(max); });

    done_button_ = new QPushButton("Calibrate", this);

    top_layout->addLayout(live_layout_);
    top_layout->addWidget(side_scroll_area_);
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

  void LiveStreamExtractionWidget::AddExtractionItem(std::unique_ptr<ExtractionImageWidget::Data> data) {
    ExtractionImageWidget* extraction_image = new ExtractionImageWidget(std::move(data), this);
    side_layout_->addWidget(extraction_image);
  }

  void LiveStreamExtractionWidget::AddLiveModeWidget(QWidget* widget) {
    live_layout_->addWidget(widget);
  }

  void LiveStreamExtractionWidget::SetCompleteButtonCallback(std::function<void()>& finish_callback) {
    connect(done_button_, &QPushButton::released, finish_callback);
  }

  void LiveStreamExtractionWidget::SignalImageAcquisition() {
    flash_widget_->resize(image_widget_->drawSize());
    flash_widget_->move(image_widget_->mapToGlobal(image_widget_->pos()));

    flash_widget_->show();
    flash_animation_->start();
  }

  std::vector<ExtractionImageWidget*> LiveStreamExtractionWidget::RemoveExtractionImagesWidgets() {
    std::vector<ExtractionImageWidget*> widgets;

    int num_widgets = side_layout_->count();
    for (int i = 0; i < num_widgets; i++) {
      ExtractionImageWidget* widget = static_cast<ExtractionImageWidget*>(side_layout_->itemAt(0)->widget());
      side_layout_->removeWidget(widget);
      widgets.push_back(widget);
    }

    return widgets;
  }
}