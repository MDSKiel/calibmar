#pragma once

#include "calibmar/core/image.h"
#include "calibmar/pose_suggestion/target_tracker.h"
#include "calibmar/readers/livestream_reader.h"

#include "ui/dialogs/stream_calibration_dialog.h"
#include "ui/utils/auto_reset_signal.h"
#include "ui/widgets/calibration_widget.h"
#include "ui/widgets/livestream_extraction_widget.h"
#include "ui/widgets/timer_bar_widget.h"

#include <opencv2/core.hpp>

namespace calibmar {

  class LivestreamCalibrationRunner {
   public:
    LivestreamCalibrationRunner(CalibrationWidget* calibration_widget, LiveStreamExtractionWidget* extraction_widget,
                                const StreamCalibrationDialog::Options& dialog_options);

    bool Run(Calibration& calibration);

   private:
    CalibrationWidget* calibration_widget_;
    LiveStreamExtractionWidget* extraction_widget_;
    QPushButton* capture_button_;
    StreamCalibrationDialog::Options dialog_options_;
    Image current_draw_image_;
    std::vector<Eigen::Vector2d> current_draw_target_points_;
    Image currently_extracted_image_;
    std::vector<Eigen::Vector2d> current_target_points_;
    // Current pixmap that is worked on, e.g. extracted and saved.
    std::unique_ptr<Pixmap> current_pixmap_;
    // Copy of the last image, used for offset diagram.
    std::unique_ptr<Pixmap> last_pixmap_;
    // Signals that a new set of extracted points is available
    std::atomic<bool> new_extraction_;
    // Signals that a image should be acquired (by button press)
    std::atomic<bool> acquire_;
    // True while extraction should be run. Used to stop extracting.
    std::atomic<bool> run_extraction_;
    // Signal the extracton to run
    AutoResetSignal extract_;
    std::unique_ptr<FeatureExtractor> extractor_;
    std::unique_ptr<TargetVisualizer> target_visualizer_;
    std::chrono::steady_clock::time_point stable_detection_start_;
    // Used for image names
    time_t run_start_;
    int image_counter_;

    void DrawLivestreamImage(std::unique_ptr<Pixmap> pixmap, int columns, int rows);
    void RunBasicExtraction(Calibration& calibration);
    void RunTimedExtraction(Calibration& calibration, TargetTracker& tracker);
    void RunPoseSuggestionExtraction(Calibration& calibration, TargetTracker& tracker, CameraModelType camera_model,
                                     const std::pair<int, int>& image_size);
    void Save(Calibration& calibration, Image image, std::unique_ptr<Pixmap> pixmap);
    void ShowCapturePossible(bool active);
  };
}