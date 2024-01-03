#include "test_widget_dialog.h"
#include "ui/widgets/image_widget.h"

#include <QtWidgets>
#include <colmap/estimators/pose.h>
#include <colmap/util/misc.h>
#include <filesystem>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "calibmar/extractors/chessboard_extractor.h"
#include "calibmar/readers/livestream_reader.h"
#include "ui/widgets/collapsible_widget.h"
#include "ui/widgets/offset_diagram_widget.h"
#include "ui/widgets/zoomable_scroll_area.h"

#include "colmap/ui/model_viewer_widget.h"

#include "ui/widgets/calibration_result_widget.h"
#include <csignal>

namespace calibmar {

  TestWidgetDialog::TestWidgetDialog(QWidget* parent) : QDialog(parent) {
    QVBoxLayout* layout = new QVBoxLayout(this);

    // pixmap_.Read(R"("")");
    // Eigen::Matrix3d camera_mat;
    // camera_mat << 1024, 0, 1024, 0, 1024, 768, 0, 0, 1;

    // ZoomableScrollArea* area = new ZoomableScrollArea(this);
    // area->setFixedHeight(500);
    // OffsetDiagramWidget* widget = new OffsetDiagramWidget({0.0132145, 2.33874e-05, -0.0298957}, camera_mat, pixmap_, this);
    // area->setWidget(widget);
    // area->widget()->resize(pixmap_.Width() * (800.0 / pixmap_.Height()), 800);
    // QTimer::singleShot(0, this, [area]() { area->verticalScrollBar()->setValue(area->verticalScrollBar()->maximum()); });

    // CollapsibleWidget* w = new CollapsibleWidget("Show Displacement", this);
    // w->SetWidget(area, 500);

    // layout->addWidget(w, 0, Qt::AlignTop);

    //----------------------------------------------------------------------------------------------

    // std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_unique<colmap::Reconstruction>();
    // reconstruction->ReadBinary("/home/felix/Desktop/col_test_dia/images/");
    // // reconstruction->ReadBinary("/home/felix/Desktop/col_test_dot2");
    // options_manager_ = std::make_unique<colmap::OptionManager>();
    // options_manager_->AddAllOptions();
    // colmap::ModelViewerWidget* model_viewer_widget = new colmap::ModelViewerWidget(this, options_manager_.get());
    // model_viewer_widget->statusbar_status_label = new QLabel("0 Images - 0 Points", this);
    // model_viewer_widget->statusbar_status_label->setVisible(false);
    // model_viewer_widget->reconstruction = reconstruction;

    // // CollapsibleWidget* collapse = new CollapsibleWidget("Reconstruction", this);
    // // collapse->SetWidget(model_viewer_widget, 500);
    // model_viewer_widget->setMinimumHeight(500);
    // model_viewer_widget->setMinimumWidth(500);

    // layout->addWidget(model_viewer_widget, 0, Qt::AlignTop);

    // QTimer::singleShot(0, this, [model_viewer_widget]() { model_viewer_widget->ReloadReconstruction(); });

    // QPushButton* button = new QPushButton("refresh", this);
    // QObject::connect(button, &QPushButton::clicked, this,
    //                  [model_viewer_widget]() { model_viewer_widget->ReloadReconstruction(); });
    // layout->addWidget(button);

    //----------------------------------------------------------------------------------------------

    Calibration calibration;
    calibration.Camera().InitializeWithName(CameraModel::CameraModels().at(CameraModelType::OpenCVCameraModel).model_name, 1000,
                                            800, 600);
    calibration.Camera().SetRefracModelIdFromName(
        HousingInterface::HousingInterfaces().at(HousingInterfaceType::DoubleLayerPlanarRefractive).model_name);
    calibration.Camera().SetRefracParams({0, 0, 1, 0.12, 1, 1.44, 1.32});

    calibration.SetCalibrationRms(1.32313);
    calibration.SetIntrinsicsStdDeviations({1.1, 1.2, 1.3});

    std::vector<double> rmss;
    for (int i = 0; i < 30; i++) {
      Image img;
      img.SetName("image_asd_" + std::to_string(i));
      calibration.AddImage(img);
      rmss.push_back(sqrt(i));
    }
    calibration.SetPerViewRms(rmss);

    CalibrationResultWidget* result = new CalibrationResultWidget(calibration);
    layout->addWidget(result);
  }
}