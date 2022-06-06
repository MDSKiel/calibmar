// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICESCOLMAP_ADD_TES>T; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#define TEST_NAME "base/ray3d_test"
#include "util/math.h"
#include "util/random.h"
#include "util/testing.h"

#include "base/ray3d.h"

using namespace colmap;

BOOST_AUTO_TEST_CASE(TestDefault) {
  Ray3D ray3D;
  BOOST_CHECK_EQUAL(ray3D.ori(0), 0.0);
  BOOST_CHECK_EQUAL(ray3D.ori(1), 0.0);
  BOOST_CHECK_EQUAL(ray3D.ori(2), 0.0);
  BOOST_CHECK_EQUAL(ray3D.dir(0), 0.0);
  BOOST_CHECK_EQUAL(ray3D.dir(1), 0.0);
  BOOST_CHECK_EQUAL(ray3D.dir(2), 1.0);

  Ray3D ray3D2(Eigen::Vector3d(0.2, 0.1, 0.5), Eigen::Vector3d(1.0, -0.4, 3.0));
  BOOST_CHECK_EQUAL(ray3D2.ori(0), 0.2);
  BOOST_CHECK_EQUAL(ray3D2.ori(1), 0.1);
  BOOST_CHECK_EQUAL(ray3D2.ori(2), 0.5);
  BOOST_CHECK_EQUAL(ray3D2.dir(0),
                    Eigen::Vector3d(1.0, -0.4, 3.0).normalized()(0));
  BOOST_CHECK_EQUAL(ray3D2.dir(1),
                    Eigen::Vector3d(1.0, -0.4, 3.0).normalized()(1));
  BOOST_CHECK_EQUAL(ray3D2.dir(2),
                    Eigen::Vector3d(1.0, -0.4, 3.0).normalized()(2));

  BOOST_CHECK_EQUAL(ray3D2.dir.norm(), 1.0);

  Eigen::Vector3d point = Eigen::Vector3d(0.2, 0.1, 0.5) +
                          0.3 * Eigen::Vector3d(1.0, -0.4, 3.0).normalized();

  BOOST_CHECK_EQUAL(ray3D2.At(0.3)(0), point(0));
  BOOST_CHECK_EQUAL(ray3D2.At(0.3)(1), point(1));
  BOOST_CHECK_EQUAL(ray3D2.At(0.3)(2), point(2));

  Eigen::Vector3d point2 = Eigen::Vector3d(0.2, 0.1, 0.5) -
                           0.7 * Eigen::Vector3d(1.0, -0.4, 3.0).normalized();
  BOOST_CHECK_EQUAL(ray3D2.At(-0.7)(0), point2(0));
  BOOST_CHECK_EQUAL(ray3D2.At(-0.7)(1), point2(1));
  BOOST_CHECK_EQUAL(ray3D2.At(-0.7)(2), point2(2));
}

BOOST_AUTO_TEST_CASE(TestComputeRefraction) {
  size_t num_tests = 10000;
  for (size_t t = 0; t < num_tests; t++) {
    // random n1 and n2;
    double n1 = RandomReal(0.5, 2.0);
    double n2 = RandomReal(0.5, 2.0);
    Eigen::Vector3d normal(0.0, 0.0, -1.0);
    Eigen::Vector3d v(0.0, 0.0, 1.0);
    double angle_max = 80.0;
    if (n1 > n2) {
      // compute critical angle to avoid total reflection case
      double ca = RadToDeg(std::asin(n2 / n1));
      angle_max = ca;
    }
    double angle = RandomReal(-angle_max, angle_max);
    Eigen::Matrix3d R =
        Eigen::AngleAxisd(DegToRad(angle), Eigen::Vector3d(0.0, 1.0, 0.0))
            .toRotationMatrix();
    v = R * v;

    double theta1 = std::acos(v.dot(-1.0 * normal));
    ComputeRefraction(normal, n1, n2, &v);
    double theta2 = std::acos(v.dot(normal));
    // Evaluate Snell's law
    BOOST_CHECK_LT(std::abs(sin(theta1) * n1 - sin(theta2) * n2), 1e-6);
  }
}

BOOST_AUTO_TEST_CASE(TestRaySphereIntersectionNaive) {
  Eigen::Vector3d ori(0.0, 0.0, 0.0);
  Eigen::Vector3d dir(0.0, 0.0, 1.0);
  Eigen::Vector3d center(0.0, 0.0, 2.0);
  double r = 1.0;
  double dmin, dmax;
  int num_points = RaySphereIntersection(ori, dir, center, r, &dmin, &dmax);
  Eigen::Vector3d point1 = ori + dmin * dir;
  Eigen::Vector3d point2 = ori + dmax * dir;
  BOOST_CHECK_EQUAL(num_points, 2);
  BOOST_CHECK_EQUAL(point1(0), 0.0);
  BOOST_CHECK_EQUAL(point1(1), 0.0);
  BOOST_CHECK_EQUAL(point1(2), 1.0);
  BOOST_CHECK_EQUAL(point2(0), 0.0);
  BOOST_CHECK_EQUAL(point2(1), 0.0);
  BOOST_CHECK_EQUAL(point2(2), 3.0);
}

BOOST_AUTO_TEST_CASE(TestRayPlaneIntersectionNaive) {
  Eigen::Vector3d ori(0.0, 0.0, 0.0);
  Eigen::Vector3d dir(0.0, 0.0, 1.0);
  Eigen::Vector3d normal(0.0, 0.0, -1.0);
  double distance = 1.0;
  double d;
  bool is_intersect = RayPlaneIntersection(ori, dir, normal, distance, &d);
  Eigen::Vector3d point = ori + d * dir;
  BOOST_CHECK(is_intersect);
  BOOST_CHECK_EQUAL(point(0), 0.0);
  BOOST_CHECK_EQUAL(point(1), 0.0);
  BOOST_CHECK_EQUAL(point(2), 1.0);
}

BOOST_AUTO_TEST_CASE(TestPointToRayDistanceNaive) {
  Eigen::Vector3d ori(0.0, 0.0, 0.0);
  Eigen::Vector3d dir(0.0, 0.0, 1.0);
  Eigen::Vector3d point1(0.0, 0.0, 2.0);
  Eigen::Vector3d point2(0.0, 1.5, -3.5);
  Eigen::Vector3d point3(4.2, 0.0, 3.7);

  const double d1 = PointToRayDistance(point1, ori, dir);
  const double d2 = PointToRayDistance(point2, ori, dir);
  const double d3 = PointToRayDistance(point3, ori, dir);

  BOOST_CHECK_EQUAL(d1, 0.0);
  BOOST_CHECK_EQUAL(d2, 1.5);
  BOOST_CHECK_EQUAL(d3, 4.2);
}
