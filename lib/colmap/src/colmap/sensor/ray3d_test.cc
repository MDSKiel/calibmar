#include "colmap/sensor/ray3d.h"

#include "colmap/math/math.h"
#include "colmap/math/random.h"

#include <gtest/gtest.h>

namespace colmap {

TEST(Ray3d, RayBasic) {
  Ray3D ray3D;
  EXPECT_EQ(ray3D.ori(0), 0.0);
  EXPECT_EQ(ray3D.ori(1), 0.0);
  EXPECT_EQ(ray3D.ori(2), 0.0);
  EXPECT_EQ(ray3D.dir(0), 0.0);
  EXPECT_EQ(ray3D.dir(1), 0.0);
  EXPECT_EQ(ray3D.dir(2), 1.0);

  Ray3D ray3D2(Eigen::Vector3d(0.2, 0.1, 0.5), Eigen::Vector3d(1.0, -0.4, 3.0));
  EXPECT_EQ(ray3D2.ori(0), 0.2);
  EXPECT_EQ(ray3D2.ori(1), 0.1);
  EXPECT_EQ(ray3D2.ori(2), 0.5);
  EXPECT_EQ(ray3D2.dir(0), Eigen::Vector3d(1.0, -0.4, 3.0).normalized()(0));
  EXPECT_EQ(ray3D2.dir(1), Eigen::Vector3d(1.0, -0.4, 3.0).normalized()(1));
  EXPECT_EQ(ray3D2.dir(2), Eigen::Vector3d(1.0, -0.4, 3.0).normalized()(2));

  EXPECT_EQ(ray3D2.dir.norm(), 1.0);

  Eigen::Vector3d point = Eigen::Vector3d(0.2, 0.1, 0.5) +
                          0.3 * Eigen::Vector3d(1.0, -0.4, 3.0).normalized();

  EXPECT_EQ(ray3D2.At(0.3)(0), point(0));
  EXPECT_EQ(ray3D2.At(0.3)(1), point(1));
  EXPECT_EQ(ray3D2.At(0.3)(2), point(2));

  Eigen::Vector3d point2 = Eigen::Vector3d(0.2, 0.1, 0.5) -
                           0.7 * Eigen::Vector3d(1.0, -0.4, 3.0).normalized();
  EXPECT_EQ(ray3D2.At(-0.7)(0), point2(0));
  EXPECT_EQ(ray3D2.At(-0.7)(1), point2(1));
  EXPECT_EQ(ray3D2.At(-0.7)(2), point2(2));
}

TEST(Ray3D, ComputeRefraction) {
  size_t num_tests = 10000;
  for (size_t t = 0; t < num_tests; t++) {
    // random n1 and n2;
    double n1 = RandomUniformReal(0.5, 2.0);
    double n2 = RandomUniformReal(0.5, 2.0);
    Eigen::Vector3d normal(0.0, 0.0, 1.0);
    Eigen::Vector3d v(0.0, 0.0, 1.0);
    double angle_max = 80.0;
    if (n1 > n2) {
      // compute critical angle to avoid total reflection case
      double ca = RadToDeg(std::asin(n2 / n1));
      angle_max = ca;
    }
    double angle = RandomUniformReal(-angle_max, angle_max);
    Eigen::Matrix3d R =
        Eigen::AngleAxisd(DegToRad(angle), Eigen::Vector3d(0.0, 1.0, 0.0))
            .toRotationMatrix();
    v = R * v;

    double theta1 = std::acos(v.dot(normal));
    ComputeRefraction(normal, n1, n2, &v);
    double theta2 = std::acos(v.dot(normal));
    // Evaluate Snell's law
    EXPECT_LT(std::abs(sin(theta1) * n1 - sin(theta2) * n2), 1e-6);
  }
}

TEST(Ray3D, RaySphereIntersectionNaive) {
  Eigen::Vector3d ori(0.0, 0.0, 0.0);
  Eigen::Vector3d dir(0.0, 0.0, 1.0);
  Eigen::Vector3d center(0.0, 0.0, 2.0);
  double r = 1.0;
  double dmin, dmax;
  int num_points = RaySphereIntersection(ori, dir, center, r, &dmin, &dmax);
  Eigen::Vector3d point1 = ori + dmin * dir;
  Eigen::Vector3d point2 = ori + dmax * dir;
  EXPECT_EQ(num_points, 2);
  EXPECT_EQ(point1(0), 0.0);
  EXPECT_EQ(point1(1), 0.0);
  EXPECT_EQ(point1(2), 1.0);
  EXPECT_EQ(point2(0), 0.0);
  EXPECT_EQ(point2(1), 0.0);
  EXPECT_EQ(point2(2), 3.0);
}

TEST(Ray3D, RayPlaneIntersectionNaive) {
  Eigen::Vector3d ori(0.0, 0.0, 0.0);
  Eigen::Vector3d dir(0.0, 0.0, 1.0);
  Eigen::Vector3d normal(0.0, 0.0, 1.0);
  double distance = 1.0;
  double d;
  bool is_intersect = RayPlaneIntersection(ori, dir, normal, distance, &d);
  Eigen::Vector3d point = ori + d * dir;
  EXPECT_TRUE(is_intersect);
  EXPECT_EQ(point(0), 0.0);
  EXPECT_EQ(point(1), 0.0);
  EXPECT_EQ(point(2), 1.0);
}

TEST(Ray3D, PointToRayDistanceNaive) {
  Eigen::Vector3d ori(0.0, 0.0, 0.0);
  Eigen::Vector3d dir(0.0, 0.0, 1.0);
  Eigen::Vector3d point1(0.0, 0.0, 2.0);
  Eigen::Vector3d point2(0.0, 1.5, -3.5);
  Eigen::Vector3d point3(4.2, 0.0, 3.7);

  const double d1 = PointToRayDistance(point1, ori, dir);
  const double d2 = PointToRayDistance(point2, ori, dir);
  const double d3 = PointToRayDistance(point3, ori, dir);

  EXPECT_EQ(d1, 0.0);
  EXPECT_EQ(d2, 1.5);
  EXPECT_EQ(d3, 4.2);
}

}  // namespace colmap