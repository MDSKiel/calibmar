#include "calibration_result_widget.h"

#include "calibmar/core/report.h"
#include "ui/utils/heatmap.h"
#include "ui/widgets/collapsible_widget.h"
#include "ui/widgets/image_widget.h"
#include "ui/widgets/offset_diagram_widget.h"
#include "ui/widgets/zoomable_scroll_area.h"

#include <colmap/ui/model_viewer_widget.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <regex>

namespace {
  std::string cssStyle = R"(
h3 {
  margin: 0px 0px 0px 0px;
}

p {
  margin: 0px 0px 10px 0px;
}

table {
  margin: 0px 0px 10px 0px;  
}

td {
  padding-right: 15px;
}
)";

  Eigen::IOFormat htmlTableFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "</td>\n<td>", "", "<tr>\n<td>", "</td>\n</tr>\n",
                                  "<table>\n<tbody>\n", "</tbody>\n</table>");

  std::vector<std::string> Split(const std::string& input, const std::string& regex) {
    // passing -1 as the submatch index parameter performs splitting
    std::regex re(regex);
    std::sregex_token_iterator first{input.begin(), input.end(), re, -1}, last;
    return {first, last};
  }

  void FormatTableRows(std::ostream& stream, const std::vector<std::string>& first_col, const std::vector<std::string>& first_row,
                       const std::vector<double>& second_row, const std::vector<double>& third_row) {
    stream << "<table>\n<tbody>\n<tr>"
           << "<td><b>" << first_col[0] << ":</b></td>\n";
    for (auto& value : first_row) {
      stream << "<td><b>" << value << "</b></td>\n";
    }
    stream << "</tr>\n<tr>";
    // parameter values
    stream << "<td><b>" << first_col[1] << "</b></td>";
    for (auto& value : second_row) {
      stream << "<td>" << value << "</td>\n";
    }
    stream << "</tr>\n";
    // parameter std dev
    if (third_row.size() > 0) {
      stream << "<tr>\n<td><b>" << first_col[2] << "</b></td>";
      for (auto& value : third_row) {
        stream << "<td>" << value << "</td>\n";
      }
      stream << "</tr>";
    }

    stream << "\n</tbody>\n</table>\n";
  }

  void GenerateResultHtml(std::ostream& stream, const calibmar::Calibration& calibration) {
    const colmap::Camera& camera = calibration.Camera();
    // camera model
    std::string header = camera.IsCameraRefractive() ? "<h3>Camera &amp; Housing Model:</h3>" : "<h3>Camera Model:</h3>";
    stream << header << std::endl << "<p>" << camera.ModelName();
    if (camera.IsCameraRefractive()) {
      stream << " " << camera.RefracModelName();
    }
    stream << "</p>" << std::endl << std::endl;
    // width & height
    stream << "<h3>Width &amp; Height:</h3>" << std::endl;
    stream << "<p>" << camera.width << " " << camera.height << "</p>";
    stream << std::endl << std::endl;
    // camera matrix
    stream << "<h3>Camera Matrix:</h3>" << std::endl
           << camera.CalibrationMatrix().format(htmlTableFormat) << std::endl
           << std::endl;
    // parameter lables
    std::vector<std::string> param_names = Split(camera.ParamsInfo(), ", ");
    FormatTableRows(stream, {"Parameters", "Values", "Est. Std. Deviations:"}, param_names, camera.params,
                    calibration.IntrinsicsStdDeviations());

    // optional refractive
    if (camera.IsCameraRefractive()) {
      std::vector<std::string> housing_param_names = Split(camera.RefracParamsInfo(), ", ");
      FormatTableRows(stream, {"Housing Parameters", "Values", "Est. Std. Deviations:"},
                      Split(Split(camera.RefracParamsInfo(), "\n")[0], ", "), camera.refrac_params,
                      calibration.HousingParamsStdDeviations());
    }
    // optional stereo pose
    if (calibration.CameraToWorldStereo().has_value()) {
      const colmap::Rigid3d& cam_to_world = calibration.CameraToWorldStereo().value();
      stream << std::endl << "<h3>Stereo Pose (camera to world R|t):</h3>" << std::endl;
      stream << cam_to_world.ToMatrix().format(htmlTableFormat) << std::endl;
      stream << "<p>(Distance to origin: " << cam_to_world.translation.norm() << ")</p>" << std::endl;
    }
    // overall rms
    stream << std::endl << "<h3>Overall RMS:</h3>\n<p>" << calibration.CalibrationRms() << "</p>\n";
  }

  void GeneratePerViewHtml(std::ostream& stream, const calibmar::Calibration& calibration) {
    // per view rms & per view observation
    if (calibration.PerViewRms().size() > 0) {
      struct Stats {
        std::string name;
        double rms;
        int point_3d_count;
      };
      std::vector<Stats> stats;
      bool contains_3d_count = calibration.PerView3DPointCount().size() > 0;
      stats.reserve(calibration.Images().size());
      for (size_t i = 0; i < calibration.Images().size(); i++) {
        stats.push_back({std::filesystem::path(calibration.Image(i).Name()).filename().string(), calibration.PerViewRms()[i],
                         contains_3d_count ? calibration.PerView3DPointCount()[i] : -1});
      }
      std::sort(stats.begin(), stats.end(), [](Stats& a, Stats& b) { return a.rms > b.rms; });

      stream << std::endl
             << std::endl
             << "<h3>Per View RMS (" << stats.size() << " images, ordered descending):</h3>" << std::endl;
      stream << "<table>\n<tbody>\n";

      stream << "\n<tr>\n<td><b>Image</b></td>\n<td><b>RMS</b></td>\n";
      if (contains_3d_count) {
        stream << "<td><b>Observed 3D Points</b></td>\n";
      }
      stream << "</tr>\n";

      for (const auto& stat : stats) {
        stream << "\n<tr>\n<td>" << stat.name << "</td>\n<td>" << stat.rms << "</td>\n";
        // optionally add 3d point count if it exists
        if (contains_3d_count) {
          stream << "<td>" << stat.point_3d_count << "</td>\n";
        }
        stream << "</tr>\n";
      }

      stream << "\n</tbody>\n</table>\n";
    }
  }
}

namespace calibmar {
  CalibrationResultWidget::CalibrationResultWidget(Calibration& calibration, std::unique_ptr<Pixmap> offset_visu_pixmap,
                                                   std::shared_ptr<colmap::Reconstruction> reconstruction, QWidget* parent)
      : QWidget(parent), offset_visu_pixmap_(std::move(offset_visu_pixmap)) {
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    // Result report
    std::stringstream result_stream;
    GenerateResultHtml(result_stream, calibration);
    QTextDocument* doc = new QTextDocument(this);
    doc->setDefaultStyleSheet(QString::fromStdString(cssStyle));
    QString text = QString::fromStdString(result_stream.str());
    result_text_ = new QTextEdit(this);
    result_text_->setDocument(doc);
    result_text_->setHtml(text);
    result_text_->setReadOnly(true);
    result_text_->setFrameStyle(QFrame::NoFrame);
    layout->addWidget(result_text_);
    layout->addStretch();

    int target_height = 500;

    if (calibration.PerViewRms().size() > 0) {
      // Preview stats
      std::stringstream per_view_stream;
      GeneratePerViewHtml(per_view_stream, calibration);
      doc = new QTextDocument(this);
      doc->setDefaultStyleSheet(QString::fromStdString(cssStyle));
      QString text = QString::fromStdString(per_view_stream.str());
      QTextEdit* per_view_report = new QTextEdit(this);
      per_view_report->setDocument(doc);
      per_view_report->setHtml(text);
      per_view_report->setReadOnly(true);
      per_view_report->setFrameStyle(QFrame::NoFrame);

      CollapsibleWidget* per_view_collapse = new CollapsibleWidget("Per View Statistics", nullptr, this);
      per_view_collapse->SetWidget(per_view_report, target_height);
      layout->addWidget(per_view_collapse);
    }

    ZoomableScrollArea* heatmap_area = new ZoomableScrollArea(this);
    heatmap_area->setFrameShape(QFrame::Shape::NoFrame);
    ImageWidget* image = new ImageWidget(this);
    heatmap_area->setWidget(image);
    heatmap_area->widget()->resize(QSize(calibration.Camera().width, calibration.Camera().height)
                                       .scaled(calibration.Camera().width, target_height, Qt::AspectRatioMode::KeepAspectRatio));
    std::unique_ptr<Pixmap> heatmap = std::make_unique<Pixmap>();
    heatmap::GenerateHeatmap(calibration.Images(), {calibration.Camera().width, calibration.Camera().height}, *heatmap);
    image->SetImage(std::move(heatmap));
    CollapsibleWidget* heatmap_collapse = new CollapsibleWidget("Heatmap", nullptr, this);
    heatmap_collapse->SetWidget(heatmap_area, target_height);
    layout->addWidget(heatmap_collapse);

    // only show offset diagramm with dome port
    if (calibration.Camera().refrac_model_id == colmap::DomePort::refrac_model_id && offset_visu_pixmap_) {
      std::vector<double>& params = calibration.Camera().refrac_params;
      OffsetDiagramWidget* offset_widget = new OffsetDiagramWidget(
          Eigen::Vector3d(params[0], params[1], params[2]), calibration.Camera().CalibrationMatrix(), *offset_visu_pixmap_, this);

      ZoomableScrollArea* area = new ZoomableScrollArea(this);
      area->setWidget(offset_widget);
      area->widget()->resize(offset_visu_pixmap_->Width() * (800.0 / offset_visu_pixmap_->Height()), 800);
      QTimer::singleShot(50, area, [area]() { area->verticalScrollBar()->setValue(area->verticalScrollBar()->maximum()); });
      CollapsibleWidget* displacement_collapse = new CollapsibleWidget("Show Displacement", nullptr, this);
      displacement_collapse->SetWidget(area, 500);

      layout->addWidget(displacement_collapse);
    }

    if (reconstruction != nullptr) {
      options_manager_ = std::make_unique<colmap::OptionManager>();
      colmap::ModelViewerWidget* model_viewer_widget = new colmap::ModelViewerWidget(this, options_manager_.get());
      model_viewer_widget->statusbar_status_label = new QLabel("0 Images - 0 Points", this);
      model_viewer_widget->statusbar_status_label->setVisible(false);
      model_viewer_widget->reconstruction = reconstruction;
      CollapsibleWidget* reconstruction_collapse = new CollapsibleWidget(
          "Reconstruction",
          [model_viewer_widget](bool visible) {
        if (visible) {
          model_viewer_widget->ReloadReconstruction();
        }
          },
          this);
      reconstruction_collapse->SetWidget(model_viewer_widget, 800);
      layout->addWidget(reconstruction_collapse);
    }
  }

  CalibrationResultWidget::CalibrationResultWidget(const std::string& message, QWidget* parent) : QWidget(parent) {
    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    QString text = QString::fromStdString(message);
    result_text_ = new QTextEdit(this);
    result_text_->setWordWrapMode(QTextOption::NoWrap);
    result_text_->setFontFamily("Courier New");
    result_text_->setPlainText(text);
    result_text_->setReadOnly(true);
    result_text_->setFrameStyle(QFrame::NoFrame);
    layout->addWidget(result_text_);
  }

  void CalibrationResultWidget::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);

    result_text_->setFixedHeight(result_text_->document()->size().height() + 20);
  }
}