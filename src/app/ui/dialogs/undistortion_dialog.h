#pragma once

#include "ui/utils/cancellation_token.h"
#include "ui/widgets/camera_model_selector_widget.h"

#include <QtCore>
#include <QtWidgets>
#include <optional>

namespace calibmar {

  class UndistortionDialog : public QDialog {
   public:
    UndistortionDialog(QWidget* parent = nullptr);

   private:
    void ImportParameters();
    bool StartUndistortingImages();

    QLineEdit* input_directory_edit_;
    QLineEdit* output_directory_edit_;
    CameraModelSelectorWidget* camera_model_edit_;

    std::unique_ptr<std::thread> runner_;
    std::atomic<bool> cancellation_flag_;
    CancellationToken cancel_;
  };
}
