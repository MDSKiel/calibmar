#include "housing_diagram.h"

#include <Eigen/Geometry>

#include "math.h"

namespace {
  Eigen::Vector3i color_water{150, 150, 255};
  Eigen::Vector3i color_glass{200, 200, 200};

  Eigen::Matrix3d ToTranslationMatrix(const Eigen::Vector2d& translation) {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
    mat.block<2, 1>(0, 2) = translation;
    return mat;
  }

  Eigen::Vector3d ToHesse(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    auto dir = p2 - p1;
    if (dir.norm() < 0.0000001) {
      return Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN()};
    }
    Eigen::Vector2d n(-dir.y(), dir.x());
    n.normalize();

    Eigen::Vector3d line;
    line.head(2) = n;
    line.z() = -n.dot(p1);
    return line;
  }

  QRectF CalculateHandle(const Eigen::Matrix3d& transformation, const Eigen::Vector2d& handle_center, int size) {
    Eigen::Vector2d transformed_center = (transformation * handle_center.homogeneous()).hnormalized();

    QPointF tl(transformed_center.x() - size, transformed_center.y() - size);
    QPointF br(transformed_center.x() + size, transformed_center.y() + size);
    return QRectF(tl, br);
  }
}

namespace calibmar {
  HousingDiagram::HousingDiagram(const Eigen::Matrix3d& transformation) : transformation_(transformation) {}

  void HousingDiagram::Scale(double factor) {
    Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
    scale.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * factor;
    transformation_ = scale;
  }

  void HousingDiagram::SetHandle(const Eigen::Vector2i& start, const Eigen::Vector2i& end, int handle_size) {
    SetHandleStart(start);
    SetHandleEnd(end);
    handle_size_ = handle_size;
  }

  void HousingDiagram::SetHandleStart(const Eigen::Vector2i& start) {
    handle_start_ = (transformation_.inverse() * start.cast<double>().homogeneous()).hnormalized();
  }

  void HousingDiagram::SetHandleEnd(const Eigen::Vector2i& end) {
    handle_end_ = (transformation_.inverse() * end.cast<double>().homogeneous()).hnormalized();
  }

  void HousingDiagram::InitCamera(const Eigen::Vector2d& center, double fov_degrees) {
    SetFOVDegrees(fov_degrees);
    camera_center_ = center;
  }

  void HousingDiagram::InitHandle(const Eigen::Vector2i& start, const Eigen::Vector2i& end, int handle_size) {
    handle_start_ = start.cast<double>();
    handle_end_ = end.cast<double>();
    handle_size_ = handle_size;
  }

  void HousingDiagram::SetRayCount(int ray_count) {
    ray_count_ = ray_count;
  }

  void HousingDiagram::SetFOVDegrees(double fov_degrees) {
    if (fov_degrees < 5 || fov_degrees > 170) {
      return;
    }
    fov_degrees_ = fov_degrees;
  }

  void HousingDiagram::InitSize(const std::pair<int, int>& size) {
    size_ = size;
  }

  void HousingDiagram::InitInterfaceThickness(double thickness) {
    interface_thickness_ = thickness;
  }

  void HousingDiagram::SetRefractiveIndexAir(double na) {
    na_ = na;
  }

  void HousingDiagram::SetRefractiveIndexGlass(double ng) {
    ng_ = ng;
  }

  void HousingDiagram::SetRefractiveIndexWater(double nw) {
    nw_ = nw;
  }

  void HousingDiagram::Draw(QPainter* painter) {
    for (const auto& poly : polygons_) {
      std::vector<QPointF> pts;
      pts.reserve(poly.points.size());
      for (const auto& pt : poly.points) {
        Eigen::Vector2d transformed = (transformation_ * pt.homogeneous()).hnormalized();
        pts.push_back(QPointF(transformed.x(), transformed.y()));
      }

      painter->setPen(Qt::PenStyle::NoPen);
      painter->setBrush(QBrush(QColor(poly.RGB.x(), poly.RGB.y(), poly.RGB.z())));
      painter->drawPolygon(pts.data(), pts.size());
    }

    for (const PartialCircle& circle : circles_) {
      Eigen::Vector2d circle_center = (transformation_ * circle.center.homogeneous()).hnormalized();
      double radius_outer = (circle.radius_inner + circle.thickness) * transformation_.coeff(0);
      double radius_inner = circle.radius_inner * transformation_.coeff(0);

      QRectF outer{circle_center.x() - radius_outer, circle_center.y() - radius_outer, radius_outer * 2, radius_outer * 2};
      QRectF inner{circle_center.x() - radius_inner, circle_center.y() - radius_inner, radius_inner * 2, radius_inner * 2};
      int qtAngle = (circle.startAngleDeg - 90) * -16;
      int qtAngleSpan = (circle.endAngleDeg - circle.startAngleDeg) * -16;

      // circle color
      painter->setPen(Qt::PenStyle::NoPen);
      painter->setBrush(QBrush(QColor(circle.RGB.x(), circle.RGB.y(), circle.RGB.z())));
      painter->drawChord(outer, qtAngle, qtAngleSpan);
      // mask with background
      painter->setBrush(painter->background());
      painter->drawChord(inner, qtAngle, qtAngleSpan);

      // outer line
      painter->setPen(QPen(QBrush(QColor(0, 0, 0)), 1));
      painter->drawArc(outer, qtAngle, qtAngleSpan);
      // inner line
      painter->drawArc(inner, qtAngle, qtAngleSpan);
    }

    for (const auto& line : lines_) {
      Eigen::Vector2d transformed_from = (transformation_ * line.from.homogeneous()).hnormalized();
      Eigen::Vector2d transformed_to = (transformation_ * line.to.homogeneous()).hnormalized();
      QPointF p1(transformed_from.x(), transformed_from.y());
      QPointF p2(transformed_to.x(), transformed_to.y());

      painter->setPen(QPen(QBrush(QColor(line.RGB.x(), line.RGB.y(), line.RGB.z())), line.thickness));
      painter->drawLine(p1, p2);
    }

    // handles
    Eigen::Vector2d transformed_start = (transformation_ * handle_start_.homogeneous()).hnormalized();
    Eigen::Vector2d transformed_end = (transformation_ * handle_end_.homogeneous()).hnormalized();
    painter->setPen(QPen(QBrush(QColor(0, 0, 0)), 1));
    painter->drawLine(transformed_start.x(), transformed_start.y(), transformed_end.x(), transformed_end.y());
    painter->setBrush(QBrush(QColor(0, 0, 255)));
    QRectF handle = CalculateHandle(transformation_, handle_start_, handle_size_);
    painter->drawRect(handle);
    handle = CalculateHandle(transformation_, handle_end_, handle_size_);
    painter->drawRect(handle);
  }

  void HousingDiagram::GetCurrentHandles(QRectF* start, QRectF* end) {
    *start = CalculateHandle(transformation_, handle_start_, handle_size_);
    *end = CalculateHandle(transformation_, handle_end_, handle_size_);
  }

  Eigen::Matrix3d HousingDiagram::GetTransformation() {
    return transformation_;
  }

  std::vector<Eigen::Vector2d> HousingDiagram::ConstructCameraRays() {
    std::vector<Eigen::Vector2d> dirs;
    Eigen::Vector2d& camera_origin = std::get<0>(camera_pts_);
    Eigen::Vector2d& camera_corner = std::get<1>(camera_pts_);
    Eigen::Vector2d sensor_vec = std::get<2>(camera_pts_) - std::get<1>(camera_pts_);
    sensor_vec /= ray_count_;

    for (int i = 0; i <= ray_count_; i++) {
      dirs.push_back((camera_corner + i * sensor_vec) - camera_origin);
    }

    return dirs;
  }

  Eigen::Vector2d HousingDiagram::Intersect(const Line& one, const Line& two) {
    return Intersect(ToHesse(one.from, one.to), (ToHesse(two.from, two.to)));
  }

  Eigen::Vector2d HousingDiagram::Intersect(const Eigen::Vector3d& one, const Eigen::Vector3d& two) {
    Eigen::Vector3d intersect = one.cross(two);
    if (abs(intersect.z()) < 0.0000001) {
      return Eigen::Vector2d{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }
    return intersect.hnormalized();
  }

  Eigen::Vector2d HousingDiagram::Intersect(const Eigen::Vector2d& origin, const Eigen::Vector2d& dir,
                                            const Eigen::Vector2d& center, double radius) {
    Eigen::Vector2d vec = dir.normalized();
    Eigen::Vector2d L = center - origin;
    double tca = L.dot(vec);
    double d2 = L.dot(L) - tca * tca;
    double r2 = radius * radius;
    if (d2 > r2) {
      return Eigen::Vector2d{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }
    double thc = sqrt(r2 - d2);
    return origin + (tca + thc) * vec;
  }

  Eigen::Vector2d HousingDiagram::Refract(const Eigen::Vector2d& vec, const Eigen::Vector2d& normal, double n1, double n2) {
    double r = n1 / n2;
    double c = normal.dot(vec);
    double scale = sqrt(1 - r * r * (1 - c * c)) - r * c;
    Eigen::Vector2d refracted = (r * vec) + (scale * normal);
    return refracted.normalized();
  }

  FlatPortDiagram::FlatPortDiagram(const Eigen::Matrix3d& transformation) : HousingDiagram(transformation) {}

  void FlatPortDiagram::CalculatePoints() {
    lines_.clear();
    polygons_.clear();
    circles_.clear();

    Eigen::Vector2d interface_normal = (handle_end_ - handle_start_).normalized();

    Eigen::Vector3d line_left{1, 0, 0};
    Eigen::Vector3d line_right{1, 0, static_cast<double>(-size_.first)};
    Eigen::Vector3d line_top{0, 1, 0};
    Eigen::Vector3d line_bottom{0, 1, static_cast<double>(-size_.second)};

    Line interface_close;
    Eigen::Vector3d interface_line_close;
    interface_line_close.head(2) = interface_normal;
    interface_line_close.z() = -interface_normal.dot(handle_start_) + interface_thickness_;
    interface_close = {Intersect(interface_line_close, line_left), Intersect(interface_line_close, line_right)};

    Line interface_far;
    Eigen::Vector3d interface_line_far;
    interface_line_far.head(2) = interface_normal;
    interface_line_far.z() = -interface_normal.dot(handle_start_);
    interface_far = {Intersect(interface_line_far, line_left), Intersect(interface_line_far, line_right)};

    // water & interface
    std::vector<Eigen::Vector2d> water_poly{interface_far.from, interface_far.to, {size_.first, size_.second}, {0, size_.second}};
    std::vector<Eigen::Vector2d> glass_poly{interface_far.from, interface_far.to, interface_close.to, interface_close.from};

    polygons_.push_back({glass_poly, color_glass});
    polygons_.push_back({water_poly, color_water});
    lines_.push_back(interface_close);
    lines_.push_back(interface_far);

    // camera
    double focal_length = sensor_width_ / (2 * tan((fov_degrees_ * (M_PI / 180)) / 2));
    camera_pts_ = {camera_center_, camera_center_ + Eigen::Vector2d{sensor_width_ / 2, focal_length},
                   camera_center_ + Eigen::Vector2d{sensor_width_ / -2, focal_length}};
    lines_.push_back({std::get<0>(camera_pts_), std::get<1>(camera_pts_), 2});
    lines_.push_back({std::get<1>(camera_pts_), std::get<2>(camera_pts_), 2});
    lines_.push_back({std::get<2>(camera_pts_), std::get<0>(camera_pts_), 2});

    std::vector<Eigen::Vector2d> ray_dirs = ConstructCameraRays();
    const Eigen::Vector2d& camera_origin = camera_center_;

    // rays
    for (const auto& dir : ray_dirs) {
      Eigen::Vector3d ray_line = ToHesse(camera_origin, camera_origin + dir);
      Eigen::Vector2d intersect_close = Intersect(ray_line, interface_line_close);
      if (intersect_close.hasNaN()) {
        lines_.push_back({camera_origin, Intersect(ray_line, line_bottom)});
        continue;
      }
      lines_.push_back({camera_origin, intersect_close});

      Eigen::Vector2d dir_glass = Refract(dir.normalized(), interface_normal, na_, ng_);
      Eigen::Vector2d intersect_far = Intersect(interface_line_far, ToHesse(intersect_close, intersect_close + dir_glass));
      if (intersect_far.hasNaN()) {
        continue;
      }

      lines_.push_back({intersect_close, intersect_far});
      Eigen::Vector2d dir_water = Refract(dir_glass, interface_normal, ng_, nw_);
      Eigen::Vector3d line_water = ToHesse(intersect_far, intersect_far + dir_water);
      lines_.push_back({intersect_far, Intersect(line_water, line_bottom)});
      lines_.push_back({intersect_far, Intersect(line_water, line_top), 1, {150, 150, 255}});
    }

    // axis
    Eigen::Vector3d axis_line = ToHesse(camera_center_, camera_center_ + interface_normal);
    lines_.push_back({Intersect(axis_line, line_top), Intersect(axis_line, line_bottom), 1, {255, 0, 0}});
  }

  DomePortDiagram::DomePortDiagram(const Eigen::Matrix3d& transformation) : HousingDiagram(transformation) {}

  void DomePortDiagram::CalculatePoints() {
    lines_.clear();
    polygons_.clear();
    circles_.clear();

    Eigen::Vector2d dome_center = handle_start_;
    double dome_radius = (handle_end_ - handle_start_).norm();
    Eigen::Vector3d line_top{0, 1, 0};
    Eigen::Vector3d line_bottom{0, 1, static_cast<double>(-size_.second)};

    // water & interface
    Polygon water{{{0, dome_center.y()}, {size_.first, dome_center.y()}, {size_.first, size_.second}, {0, size_.second}},
                  color_water};
    PartialCircle dome{dome_center, dome_radius, 90, 270, interface_thickness_, color_glass};
    polygons_.push_back(water);
    circles_.push_back(dome);

    // camera
    double focal_length = sensor_width_ / (2 * tan((fov_degrees_ * (M_PI / 180)) / 2));
    camera_pts_ = {camera_center_, camera_center_ + Eigen::Vector2d{sensor_width_ / 2, focal_length},
                   camera_center_ + Eigen::Vector2d{sensor_width_ / -2, focal_length}};
    lines_.push_back({std::get<0>(camera_pts_), std::get<1>(camera_pts_), 2});
    lines_.push_back({std::get<1>(camera_pts_), std::get<2>(camera_pts_), 2});
    lines_.push_back({std::get<2>(camera_pts_), std::get<0>(camera_pts_), 2});

    std::vector<Eigen::Vector2d> ray_dirs = ConstructCameraRays();
    const Eigen::Vector2d& camera_origin = camera_center_;

    // rays
    for (const auto& dir : ray_dirs) {
      // Eigen::Vector3d ray_line = ToHesse(camera_origin, camera_origin + dir);
      Eigen::Vector2d intersect_close = Intersect(camera_origin, dir, dome_center, dome_radius);
      if (intersect_close.hasNaN()) {
        lines_.push_back({camera_origin, Intersect(ToHesse(camera_center_, camera_center_ + dir), line_bottom)});
        continue;
      }
      lines_.push_back({camera_origin, intersect_close});

      Eigen::Vector2d dir_glass = Refract(dir.normalized(), (intersect_close - dome_center).normalized(), na_, ng_);
      Eigen::Vector2d intersect_far = Intersect(intersect_close, dir_glass, dome_center, dome_radius + interface_thickness_);
      if (intersect_far.hasNaN()) {
        continue;
      }

      lines_.push_back({intersect_close, intersect_far});
      Eigen::Vector2d dir_water = Refract(dir_glass, (intersect_far - dome_center).normalized(), ng_, nw_);
      Eigen::Vector3d line_water = ToHesse(intersect_far, intersect_far + dir_water);
      lines_.push_back({intersect_far, Intersect(line_water, line_bottom)});
      lines_.push_back({intersect_far, Intersect(line_water, line_top), 1, {150, 150, 255}});
    }

    // axis
    Eigen::Vector3d axis_line = ToHesse(camera_center_, dome_center);
    if (!axis_line.hasNaN()) {
      lines_.push_back({Intersect(axis_line, line_top), Intersect(axis_line, line_bottom), 1, {255, 0, 0}});
    }
  }
}