#pragma once

#include <QtCore>
#include <QtWidgets>
#include <chrono>

namespace calibmar {
  // timer bar used in hands free calibration
  class TimerBarWidget : public QWidget {
   public:
    TimerBarWidget(QWidget* parent = nullptr);

    void StartTimer(std::chrono::milliseconds duration);
    void StopTimer();
    void SetLabel(const QString& label);

   private:
    QLabel* label_;
    QProgressBar* progress_bar_;
    QTimer* timer_;
    std::chrono::milliseconds duration_;
    std::chrono::time_point<std::chrono::steady_clock> timer_start_;

    void UpdateTimerLabel();
  };
}