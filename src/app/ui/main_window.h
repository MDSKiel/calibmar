#pragma once

#include "calibmar/core/pixmap.h"
#include "calibmar/extractors/chessboard_extractor.h"
#include "ui/dialogs/file_calibration_dialog.h"
#include "ui/dialogs/stream_calibration_dialog.h"
#include "ui/widgets/calibration_widget.h"
#include "ui/widgets/livestream_extraction_widget.h"

#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QScrollArea>
#include <opencv2/core.hpp>
#include <vector>

namespace calibmar {
  class MainWindow : public QMainWindow {
   public:
    MainWindow(QWidget* parent = nullptr);

   private:
    void NewStreamCalibration();
    void NewFilesCalibration();
    void NewStereoFilesCalibration();
    void SaveCalibration();
    void About();
    void CreateActions();
    void BeginNewCalibration();
    void EndNewCalibration();
    void DisplayExtractionImage(const std::string& image_name, const TargetVisualizer& target_visualizer);

    QScrollArea* scroll_area_;
    QVBoxLayout* main_layout_;
    QAction* calibration_files_action_;
    QAction* calibration_stereo_files_action_;
    QAction* calibration_stream_action_;
    QAction* calibration_save_action_;
    std::string last_directory_;
    std::unique_ptr<std::thread> worker_thread_;
    std::unique_ptr<Calibration> calibration_;
    // second calibration that only exists for a stereo calibration
    std::unique_ptr<Calibration> calibration_stereo_;
    bool scrolled_to_bottom_;
  };
}
