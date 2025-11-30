#pragma once

#include <QtCore>
#include <QtWidgets>

#include "ui/utils/calibration_target_visualization.h"
#include "ui/widgets/extraction_image_widget.h"
#include "ui/widgets/extraction_images_widget.h"

namespace calibmar {

  // Widget that holds extracted images and calibration result widget
  class CalibrationWidget : public QWidget {
   public:
    enum class SortOrder { NameDesc = 0, NameAsc = 1, RmsDesc = 2, RmsAsc = 3 };

    CalibrationWidget(QWidget* parent = nullptr,
                      const std::function<void(const std::string&, const TargetVisualizer&)> double_click_callback = nullptr,
                      const std::function<const std::vector<std::string>(const SortOrder&)> sorting_callback = nullptr);

    void AddExtractionItem(QWidget* widget, const std::string& key);
    void StartCalibration();
    void EndCalibration(QWidget* calibration_result);
    void SetTargetVisualizer(std::unique_ptr<TargetVisualizer> target_visualizer);

    // Sets per view Rms coming from one or more calibration
    void SetPerViewRms(const std::vector<std::vector<Image>*>& images, const std::vector<std::vector<double>*>& per_view_rms);
    void SetPerViewRms(const std::vector<Image>& images, const std::vector<double>& per_view_rms);

    const class TargetVisualizer& TargetVisualizer();

   private:
    void SetExtractionImageOrder(int index);

    std::unique_ptr<class TargetVisualizer> target_visualizer_;  // basically only holds this for ownership reasons. Child
                                                                 // ExtractionImageWidgets use a reference to this.
    QLayout* main_layout_;
    QWidget* calibration_widget_;
    std::vector<QWidget*> sorting_widgets_;
    ExtractionImagesWidget* extraction_images_;
    std::atomic_bool calibration_ended_;
    const std::function<const std::vector<std::string>(const SortOrder&)> sorting_callback_;
  };
}
