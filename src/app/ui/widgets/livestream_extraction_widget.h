#pragma once

#include "ui/widgets/extraction_image_widget.h"
#include "ui/widgets/image_widget.h"
#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class LiveStreamExtractionWidget : public QWidget {
   public:
    LiveStreamExtractionWidget(QWidget* parent = nullptr);
    // Sets the next live stream frame
    void SetLiveStreamImage(std::unique_ptr<Pixmap> image);
    // Adds an extracted image to the calibration
    void AddExtractionItem(std::unique_ptr<ExtractionImageWidget::Data> data);
    // Adds a widget next to the live stream view (e.g. timer bar or button)
    void AddLiveModeWidget(QWidget* widget);
    // Set a callback called when calibration is to be completed
    void SetCompleteButtonCallback(std::function<void()>& finish_callback);
    // Show a small shutter animation to signal acquisition
    void SignalImageAcquisition();
    // Take ownership of all extraction images
    std::vector<ExtractionImageWidget*> RemoveExtractionImagesWidgets();

   private:
    QWidget* flash_widget_;
    QPropertyAnimation* flash_animation_;
    QPushButton* done_button_;
    ImageWidget* image_widget_;
    QScrollArea* side_scroll_area_;
    QVBoxLayout* side_layout_;
    QVBoxLayout* live_layout_;
  };
}