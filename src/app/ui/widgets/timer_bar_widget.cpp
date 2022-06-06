#include "timer_bar_widget.h"

namespace calibmar {
  TimerBarWidget::TimerBarWidget(QWidget* parent) : QWidget(parent), duration_(0) {
    QGridLayout* layout = new QGridLayout(this);
    progress_bar_ = new QProgressBar(this);
    progress_bar_->setRange(0, 100);
    progress_bar_->setFormat("");
    progress_bar_->setTextVisible(false);
    label_ = new QLabel(this);
    label_->setAlignment(Qt::AlignCenter);
    label_->setDisabled(true);

    layout->setContentsMargins(0, 0, 0, 0);
    // overlay widgets
    layout->addWidget(progress_bar_, 0, 0);
    layout->addWidget(label_, 0, 0);

    timer_ = new QTimer(this);
    timer_->setInterval(100);
    connect(timer_, &QTimer::timeout, this, &TimerBarWidget::UpdateTimerLabel);
  }

  void TimerBarWidget::StartTimer(std::chrono::milliseconds duration) {
    duration_ = duration;
    timer_start_ = std::chrono::steady_clock::now();

    progress_bar_->setValue(0);

    if (!timer_->isActive()) {
      timer_->start();
    }
  }

  void TimerBarWidget::StopTimer() {
    timer_->stop();
    progress_bar_->setValue(0);
  }

  void TimerBarWidget::SetLabel(const QString& label) {
    label_->setText(label);
  }

  void TimerBarWidget::UpdateTimerLabel() {
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();

    if (now < timer_start_ + duration_) {
      std::chrono::milliseconds time_done = std::chrono::duration_cast<std::chrono::milliseconds>(now - timer_start_);
      double ratio = static_cast<double>(time_done.count()) / duration_.count();
      progress_bar_->setValue(ratio * 100);
    }
    else {
      progress_bar_->setValue(100);
    }
  }

}