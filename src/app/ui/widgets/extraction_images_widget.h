#pragma once

#include <QtCore>
#include <QtWidgets>

#include "calibmar/extractors/chessboard_extractor.h"
#include "ui/utils/flow_layout.h"
#include "ui/widgets/extraction_image_widget.h"

namespace calibmar {
  // Displays extraction images in a flow layout and can propagate image name and cols, rows on double click (for image display)
  class ExtractionImagesWidget : public QWidget {
   public:
    ExtractionImagesWidget(QWidget* parent = nullptr,
                           const std::function<void(const std::string&, std::pair<int, int>&)> double_click_callback = nullptr);

    void AddImage(ExtractionImageWidget* widget);

   protected:
    virtual void mouseDoubleClickEvent(QMouseEvent* event) override;

   private:
    FlowLayout* main_layout_;
    const std::function<void(const std::string&, std::pair<int, int>&)> double_click_callback_;
  };
}
