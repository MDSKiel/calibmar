#include "model_explorer_dialog.h"
#include "ui/utils/render.h"
#include "ui/widgets/zoomable_scroll_area.h"

#include <colmap/math/math.h>
#include <colmap/util/misc.h>

#include <opencv2/imgproc.hpp>

#include <thread>

namespace {
  double FocalLengthFromFOV(int edge_length_px, double fov_degrees) {
    return (edge_length_px / 2.0) / tan(colmap::DegToRad(fov_degrees / 2.0));
  }

  inline bool SampleCheckerboard(double x, double y, double scale = 0.1) {
    uint32_t a = (x + 100000) / scale;
    uint32_t b = (y + 100000) / scale;

    return (a % 2) != (b % 2);
  }

  void RenderCheckerboard(calibmar::Pixmap& img, colmap::Camera& camera, double distance, double checker_size) {
    if (camera.IsCameraRefractive()) {
      Eigen::Vector3d point3D;
      Eigen::Vector3d normal{0, 0, 1.0};
      double d;
      for (int y = 0; y < img.Height(); y++) {
        for (int x = 0; x < img.Width(); x++) {
          colmap::Ray3D ray = camera.CamFromImgRefrac({x, y});
          colmap::RayPlaneIntersection(ray.ori, ray.dir, normal, distance, &d);
          point3D = ray.ori + ray.dir * d;
          img.Data().at<uchar>(y, x) = SampleCheckerboard(point3D.x(), point3D.y(), checker_size) ? (uchar)255 : (uchar)0;
        }
      }
    }
    else {
      for (int y = 0; y < img.Height(); y++) {
        for (int x = 0; x < img.Width(); x++) {
          Eigen::Vector2d cam_point = camera.CamFromImg({x, y}) * distance;
          img.Data().at<uchar>(y, x) = SampleCheckerboard(cam_point.x(), cam_point.y(), checker_size) ? (uchar)255 : (uchar)0;
        }
      }
    }
  }

  void SetupSlider(std::string row_label, double* value, double min, double max, std::mutex& lock, QGridLayout* layout,
                   std::atomic_bool& camera_changed) {
    QSlider* slider = new QSlider(Qt::Orientation::Horizontal);
    slider->setMaximum(static_cast<int>(max * 1000));
    slider->setMinimum(static_cast<int>(min * 1000));
    slider->setValue(static_cast<int>(*value * 1000));
    QLabel* value_label = new QLabel(QString::number(*value));
    value_label->setMinimumWidth(value_label->fontMetrics().horizontalAdvance("-0.000"));
    QLabel* label = new QLabel(QString::fromStdString(row_label));

    slider->connect(slider, &QSlider::valueChanged, slider, [value_label, value, &lock, &camera_changed](int v) {
      std::lock_guard lock_guard(lock);
      *value = v / 1000.0;
      value_label->setText(QString::number(*value));
      camera_changed = true;
    });

    int row = layout->rowCount();
    layout->addWidget(label, row, 0);
    layout->addWidget(slider, row, 1);
    layout->addWidget(value_label, row, 2);
  }

  void SetupHousingRotationSlider(std::string row_label, double* value, double min, double max, std::mutex& lock,
                                  QGridLayout* layout, std::atomic_bool& camera_changed, std::vector<double>& rotations,
                                  std::vector<double>& refrac_params) {
    QSlider* slider = new QSlider(Qt::Orientation::Horizontal);
    slider->setMaximum(static_cast<int>(max * 1000));
    slider->setMinimum(static_cast<int>(min * 1000));
    slider->setValue(static_cast<int>(*value));
    QLabel* value_label = new QLabel(QString::number(*value));
    value_label->setMinimumWidth(value_label->fontMetrics().horizontalAdvance("-0.000"));
    QLabel* label = new QLabel(QString::fromStdString(row_label));

    slider->connect(slider, &QSlider::valueChanged, slider,
                    [value_label, value, &lock, &camera_changed, &rotations, &refrac_params](int v) mutable {
      std::lock_guard lock_guard(lock);
      *value = v / 1000.0;
      value_label->setText(QString::number(*value));

      Eigen::Matrix3d rotation;
      rotation = Eigen::AngleAxisd(rotations[0] * M_PI, Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(rotations[1] * M_PI, Eigen::Vector3d::UnitY());

      Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
      normal = rotation * normal;
      normal.normalize();

      for (size_t i = 0; i < 3; i++) {
        refrac_params[i] = normal(i);
      }

      camera_changed = true;
    });

    int row = layout->rowCount();
    layout->addWidget(label, row, 0);
    layout->addWidget(slider, row, 1);
    layout->addWidget(value_label, row, 2);
  }
}

namespace calibmar {
  ModelExplorerDialog::ModelExplorerDialog(QWidget* parent)
      : QDialog(parent), camera_changed_(false), cancel_(false), distance_(1), fov_(50), checker_size_(0.1) {
    setWindowTitle("Camera Model Explorer");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QSplitter* splitter = new QSplitter(this);

    // image view
    ZoomableScrollArea* area = new ZoomableScrollArea();
    image_widget_ = new ImageWidget(area);
    target_image_ = std::make_unique<Pixmap>();
    target_image_->Assign(cv::Mat::zeros(500, 500, CV_8UC1));
    image_widget_->SetImage(std::move(std::make_unique<Pixmap>(target_image_->Clone())));
    area->setMinimumSize(550, 550);
    area->setWidget(image_widget_);
    // side
    QWidget* side = new QWidget();
    QVBoxLayout* side_parent_layout = new QVBoxLayout(side);

    camera_sliders_side_layout_ = new QGridLayout();
    camera_model_ = new CameraModelWidget([this]() {
      this->SetupCameraModelSliders();
      this->SetupHousingModelSliders();
      this->camera_changed_ = true;
    });
    housing_sliders_side_layout_ = new QGridLayout();
    housing_model_ = new HousingWidget([this]() {
      this->SetupCameraModelSliders();
      this->SetupHousingModelSliders();
      this->camera_changed_ = true;
    });
    housing_model_->SetHousingType({});
    camera_model_->SetCameraModel(CameraModelType::SimpleRadialCameraModel);

    common_sliders_side_layout_ = new QGridLayout();
    SetupSlider("FOV", &fov_, 20, 120, value_lock_, common_sliders_side_layout_, camera_changed_);
    SetupSlider("Distance", &distance_, 0.25, 10, value_lock_, common_sliders_side_layout_, camera_changed_);
    SetupSlider("Checker Size", &checker_size_, 0.1, 0.999, value_lock_, common_sliders_side_layout_, camera_changed_);

    side_parent_layout->addLayout(common_sliders_side_layout_);
    side_parent_layout->addWidget(camera_model_);
    side_parent_layout->addLayout(camera_sliders_side_layout_);
    side_parent_layout->addWidget(housing_model_);
    side_parent_layout->addLayout(housing_sliders_side_layout_);

    side_parent_layout->addStretch();

    splitter->addWidget(area);
    splitter->addWidget(side);
    layout->addWidget(splitter);

    worker_thread_.reset(new std::thread(&ModelExplorerDialog::RenderLoop, this));
  }

  void ModelExplorerDialog::done(int r) {
    if (worker_thread_) {
      cancel_ = true;
      worker_thread_->join();
    }
    QDialog::done(r);
  }

  void ModelExplorerDialog::SetupCameraModelSliders() {
    std::lock_guard lock_camera(value_lock_);
    CameraModelType camera_type = camera_model_->CameraModel();
    camera_ = CameraModel::InitCamera(camera_type, {target_image_->Width(), target_image_->Height()}, target_image_->Width());

    // remove existing camera side widgets
    int num_widgets = camera_sliders_side_layout_->count();
    for (int i = 0; i < num_widgets; i++) {
      QWidget* widget = camera_sliders_side_layout_->itemAt(0)->widget();
      camera_sliders_side_layout_->removeWidget(widget);
      delete widget;
    }

    std::vector<std::string> param_labels = colmap::CSVToVector<std::string>(camera_.ParamsInfo());
    param_labels.resize(camera_.params.size());  // ensure they match (which they should already)

    for (size_t param_idx : camera_.ExtraParamsIdxs()) {
      SetupSlider(param_labels[param_idx], &camera_.params[param_idx], -1, 1, value_lock_, camera_sliders_side_layout_,
                  camera_changed_);
    }
  }

  void ModelExplorerDialog::SetupHousingModelSliders() {
    std::lock_guard lock_camera(value_lock_);
    std::optional<HousingInterfaceType> housing_type = housing_model_->HousingType();

    // remove existing housing side widgets
    int num_widgets = housing_sliders_side_layout_->count();
    for (int i = 0; i < num_widgets; i++) {
      QWidget* widget = housing_sliders_side_layout_->itemAt(0)->widget();
      housing_sliders_side_layout_->removeWidget(widget);
      delete widget;
    }

    if (!housing_type.has_value()) {
      camera_.refrac_params = {};
      camera_.refrac_model_id = colmap::CameraRefracModelId::kInvalid;
      return;
    }

    QFrame* divider = new QFrame();
    divider->setFrameShape(QFrame::HLine);
    divider->setFrameShadow(QFrame::Sunken);
    housing_sliders_side_layout_->addWidget(divider, housing_sliders_side_layout_->rowCount(), 0, 1, 3);

    camera_.refrac_model_id =
        colmap::CameraRefracModelNameToId(HousingInterface::HousingInterfaces().at(*housing_type).model_name);
    if (*housing_type == HousingInterfaceType::DoubleLayerPlanarRefractive) {
      // Nx, Ny, Nz, int_dist, int_thick, na, ng, nw
      camera_.refrac_params = {0, 0, 1, 0.015, 0.005, 1.003, 1.473, 1.333};

      rotations_ = {0, 0};
      SetupHousingRotationSlider("α", &(rotations_[0]), -0.5, 0.5, value_lock_, housing_sliders_side_layout_, camera_changed_,
                                 rotations_, camera_.refrac_params);
      SetupHousingRotationSlider("β", &(rotations_[1]), -0.5, 0.5, value_lock_, housing_sliders_side_layout_, camera_changed_,
                                 rotations_, camera_.refrac_params);
    }
    else if (*housing_type == HousingInterfaceType::DoubleLayerSphericalRefractive) {
      // Cx, Cy, Cz, int_radius, int_thick, na, ng, nw
      camera_.refrac_params = {0, 0, 0, 0.015, 0.005, 1.003, 1.473, 1.333};

      SetupSlider("Cx", &camera_.refrac_params[0], -0.1, 0.1, value_lock_, housing_sliders_side_layout_, camera_changed_);
      SetupSlider("Cy", &camera_.refrac_params[1], -0.1, 0.1, value_lock_, housing_sliders_side_layout_, camera_changed_);
      SetupSlider("Cz", &camera_.refrac_params[2], -0.1, 0.1, value_lock_, housing_sliders_side_layout_, camera_changed_);
    }

    SetupSlider(*housing_type == HousingInterfaceType::DoubleLayerSphericalRefractive ? "radius" : "dist",
                &camera_.refrac_params[3], 0, 1, value_lock_, housing_sliders_side_layout_, camera_changed_);
    SetupSlider("thickness", &camera_.refrac_params[4], 0, 1, value_lock_, housing_sliders_side_layout_, camera_changed_);
    SetupSlider("na", &camera_.refrac_params[5], 1, 2, value_lock_, housing_sliders_side_layout_, camera_changed_);
    SetupSlider("ng", &camera_.refrac_params[6], 1, 2, value_lock_, housing_sliders_side_layout_, camera_changed_);
    SetupSlider("nw", &camera_.refrac_params[7], 1, 2, value_lock_, housing_sliders_side_layout_, camera_changed_);
  }

  void ModelExplorerDialog::RenderLoop() {
    while (!cancel_) {
      // Limit the rendering loop to run maximally every ~200ms
      auto end = std::chrono::system_clock::now() + std::chrono::milliseconds(200);

      colmap::Camera camera;
      double distance;
      double checker_size;
      if (camera_changed_) {
        {
          std::lock_guard lock(value_lock_);
          camera = camera_;
          camera.SetFocalLength(FocalLengthFromFOV(camera.width, fov_));
          distance = distance_;
          checker_size = checker_size_;
        }

        RenderCheckerboard(*target_image_, camera, distance, checker_size);

        // swap images
        image_widget_->setUpdatesEnabled(false);
        std::unique_ptr<Pixmap> tmp = image_widget_->TakeImage();
        image_widget_->SetImage(std::move(target_image_));
        target_image_ = std::move(tmp);
        image_widget_->setUpdatesEnabled(true);
        camera_changed_ = false;
      }
      std::this_thread::sleep_until(end);
    }
  }
}
