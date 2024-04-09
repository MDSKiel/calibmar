#pragma once

#include <Eigen/Core>
#include <QtGui>

namespace calibmar {

  class HousingDiagram {
   public:
    HousingDiagram(const Eigen::Matrix3d& transformation = Eigen::Matrix3d::Identity());
    virtual void CalculatePoints() = 0;

    void Scale(double factor);

    void InitSize(const std::pair<int, int>& size);
    void InitInterfaceThickness(double thickness);
    void InitCamera(const Eigen::Vector2d& center, double fov_degrees);
    void InitHandle(const Eigen::Vector2i& start, const Eigen::Vector2i& end, int handle_size = 10);

    void SetHandle(const Eigen::Vector2i& start, const Eigen::Vector2i& end, int handle_size = 10);
    void SetHandleStart(const Eigen::Vector2i& start);
    void SetHandleEnd(const Eigen::Vector2i& end);
    void SetRayCount(int ray_count);
    void SetFOVDegrees(double fov_degrees);
    void SetRefractiveIndexAir(double na);
    void SetRefractiveIndexGlass(double ng);
    void SetRefractiveIndexWater(double nw);

    void Draw(QPainter* painter);
    void GetCurrentHandles(QRectF* start, QRectF* end);
    Eigen::Matrix3d GetTransformation();

    ~HousingDiagram() = default;

   protected:
    struct Line {
      Eigen::Vector2d from;
      Eigen::Vector2d to;
      int thickness = 1;
      Eigen::Vector3i RGB = {0, 0, 0};
    };
    struct Polygon {
      std::vector<Eigen::Vector2d> points;
      Eigen::Vector3i RGB;
    };
    struct PartialCircle {
      Eigen::Vector2d center;
      double radius_inner;
      // start angle is at 12 o'clock, CW is positive
      double startAngleDeg;
      double endAngleDeg;
      double thickness;
      Eigen::Vector3i RGB;
    };

    std::vector<Eigen::Vector2d> ConstructCameraRays();

    Eigen::Vector2d Intersect(const Line& one, const Line& two);
    Eigen::Vector2d Intersect(const Eigen::Vector3d& one, const Eigen::Vector3d& two);
    Eigen::Vector2d Intersect(const Eigen::Vector2d& origin, const Eigen::Vector2d& dir, const Eigen::Vector2d& center,
                              double radius);
    Eigen::Vector2d Refract(const Eigen::Vector2d& vec, const Eigen::Vector2d& normal, double n1, double n2);

    std::pair<int, int> size_;
    double interface_thickness_;
    Eigen::Vector2d handle_start_;
    Eigen::Vector2d handle_end_;
    Eigen::Vector2d camera_center_;
    int handle_size_;
    std::vector<Line> lines_;
    std::vector<Polygon> polygons_;
    std::vector<PartialCircle> circles_;
    std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d> camera_pts_;
    int ray_count_;
    double fov_degrees_;
    double sensor_width_ = 70;
    Eigen::Matrix3d transformation_ = Eigen::Matrix3d::Identity();
    double na_ = 1;
    double ng_ = 1.47;
    double nw_ = 1.334;
  };

  class FlatPortDiagram : public HousingDiagram {
   public:
    FlatPortDiagram(const Eigen::Matrix3d& transformation = Eigen::Matrix3d::Identity());
    void CalculatePoints() override;
  };

  class DomePortDiagram : public HousingDiagram {
   public:
    DomePortDiagram(const Eigen::Matrix3d& transformation = Eigen::Matrix3d::Identity());
    void CalculatePoints() override;
  };

}