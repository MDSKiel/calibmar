#pragma once

#include <QtCore>
#include <QtWidgets>

#include "calibmar/extractors/chessboard_extractor.h"
#include "ui/utils/calibration_target_visualization.h"
#include "ui/utils/image_flow_layout.h"
#include "ui/widgets/extraction_image_widget.h"

namespace calibmar {
  // Displays extraction images in a flow layout and can propagate image name and cols, rows on double click (for image display)
  class ExtractionImagesWidget : public QWidget {
   public:
    ExtractionImagesWidget(
        QWidget* parent = nullptr,
        const std::function<void(const std::string&, const TargetVisualizer&)> double_click_callback = nullptr);

    void AddExtractionItem(QWidget* widget, const std::string& key);

    void SetOrder(const std::vector<std::string>& order);
    void SetPerViewRMS(const std::unordered_map<std::string, double>& per_view_rms);

   protected:
    virtual void mouseDoubleClickEvent(QMouseEvent* event) override;
    virtual void contextMenuEvent(QContextMenuEvent* event) override;

   private:
    ImageFlowLayout* main_layout_;
    const std::function<void(const std::string&, const TargetVisualizer&)> double_click_callback_;
    std::unordered_map<std::string, QWidget*> items_;
  };
}
