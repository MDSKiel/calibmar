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
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#define TEST_NAME "base/camera_non_svp_models"
#include "util/random.h"
#include "util/testing.h"

#include "base/camera_non_svp_models.h"

using namespace colmap;

template <typename CameraNonSvpModel, typename CameraModel>
void TestWorldToImageToWorld(const std::vector<double>& non_svp_params,
                             const std::vector<double>& camera_params,
                             const Eigen::Vector3d& world_point0) {
  double x, y, xx, yy;
  Eigen::Vector3d world_point1, world_point2;
  const double depth = world_point0.norm();
  CameraNonSvpModel::template WorldToImage<CameraModel, double>(
      non_svp_params.data(), camera_params.data(), world_point0, &x, &y);
  CameraNonSvpModelWorldToImage(
      CameraModel::model_id, CameraNonSvpModel::non_svp_model_id, camera_params,
      non_svp_params, world_point0, &xx, &yy);
  BOOST_CHECK_EQUAL(x, xx);
  BOOST_CHECK_EQUAL(y, yy);
  CameraNonSvpModel::template ImageToWorldPoint<CameraModel, double>(
      non_svp_params.data(), camera_params.data(), x, y, depth, &world_point1);
  CameraNonSvpModelImageToWorldPoint(
      CameraModel::model_id, CameraNonSvpModel::non_svp_model_id, camera_params,
      non_svp_params, xx, yy, depth, &world_point2);
  BOOST_CHECK_EQUAL(world_point1(0), world_point2(0));
  BOOST_CHECK_EQUAL(world_point1(1), world_point2(1));
  BOOST_CHECK_EQUAL(world_point1(2), world_point2(2));
  BOOST_CHECK_LT(std::abs(world_point1(0) - world_point0(0)), 1e-6);
  BOOST_CHECK_LT(std::abs(world_point1(1) - world_point0(1)), 1e-6);
  BOOST_CHECK_LT(std::abs(world_point1(2) - world_point0(2)), 1e-6);
  BOOST_CHECK_LT(std::abs(world_point2(0) - world_point0(0)), 1e-6);
  BOOST_CHECK_LT(std::abs(world_point2(1) - world_point0(1)), 1e-6);
  BOOST_CHECK_LT(std::abs(world_point2(2) - world_point0(2)), 1e-6);
}

template <typename CameraNonSvpModel, typename CameraModel>
void TestImageToWorldToImage(const std::vector<double>& non_svp_params,
                             const std::vector<double>& camera_params,
                             const double x0, const double y0,
                             const double depth) {
  Eigen::Vector3d world_point1;
  Eigen::Vector3d world_point2;
  double x, y;
  CameraNonSvpModel::template ImageToWorldPoint<CameraModel, double>(
      non_svp_params.data(), camera_params.data(), x0, y0, depth,
      &world_point1);
  CameraNonSvpModelImageToWorldPoint(
      CameraModel::model_id, CameraNonSvpModel::non_svp_model_id, camera_params,
      non_svp_params, x0, y0, depth, &world_point2);
  BOOST_CHECK_EQUAL(world_point1(0), world_point1(0));
  BOOST_CHECK_EQUAL(world_point1(1), world_point1(1));
  BOOST_CHECK_EQUAL(world_point1(2), world_point1(2));
  CameraNonSvpModel::template WorldToImage<CameraModel, double>(
      non_svp_params.data(), camera_params.data(), world_point1, &x, &y);
  BOOST_CHECK_LT(std::abs(x - x0), 1e-6);
  BOOST_CHECK_LT(std::abs(y - y0), 1e-6);
}

template <typename CameraNonSvpModel, typename CameraModel>
void TestModel(const std::vector<double>& non_svp_params,
               const std::vector<double>& camera_params) {
  BOOST_CHECK(CameraNonSvpModelVerifyParams(CameraNonSvpModel::non_svp_model_id,
                                            non_svp_params));

  BOOST_CHECK_EQUAL(
      CameraNonSvpModelParamsInfo(CameraNonSvpModel::non_svp_model_id),
      CameraNonSvpModel::params_info);
  BOOST_CHECK_EQUAL(
      CameraNonSvpModelNumParams(CameraNonSvpModel::non_svp_model_id),
      CameraNonSvpModel::num_params);

  BOOST_CHECK(
      ExistsCameraNonSvpModelWithName(CameraNonSvpModel::non_svp_model_name));
  BOOST_CHECK(!ExistsCameraNonSvpModelWithName(
      CameraNonSvpModel::non_svp_model_name + "FOO"));

  BOOST_CHECK(
      ExistsCameraNonSvpModelWithId(CameraNonSvpModel::non_svp_model_id));
  BOOST_CHECK(!ExistsCameraNonSvpModelWithId(
      CameraNonSvpModel::non_svp_model_id + 123456789));

  BOOST_CHECK_EQUAL(
      CameraNonSvpModelNameToId(CameraNonSvpModel::non_svp_model_name),
      CameraNonSvpModel::non_svp_model_id);
  BOOST_CHECK_EQUAL(
      CameraNonSvpModelIdToName(CameraNonSvpModel::non_svp_model_id),
      CameraNonSvpModel::non_svp_model_name);

  for (double x = -0.5; x <= 0.5; x += 0.1) {
    for (double y = -0.5; y <= 0.5; y += 0.1) {
      for (double z = 0.5; z <= 5.5; z += 0.2) {
        Eigen::Vector3d world_point(x, y, z);
        TestWorldToImageToWorld<CameraNonSvpModel, CameraModel>(
            non_svp_params, camera_params, world_point);
      }
    }
  }

  for (double x = 0.0; x < 5568; x += 50) {
    for (double y = 0.0; y < 4176; y += 50) {
      const double depth = RandomReal(0.2, 10.0);
      TestImageToWorldToImage<CameraNonSvpModel, CameraModel>(
          non_svp_params, camera_params, x, y, depth);
    }
  }
}

BOOST_AUTO_TEST_CASE(TestDoubleLayerSphericalRefractiveInterfaceCase1) {
  std::vector<double> camera_params = {3200.484,
                                       3200.917,
                                       2790.172,
                                       2108.726,
                                       -0.233072717236466,
                                       0.065022474710061,
                                       1.008118149931866e-06,
                                       -2.863880315774651e-05};
  std::vector<double> non_svp_params = {
      0.00042007, 0.00366894, 0.0283927, 0.05, 0.007, 1.003, 1.473, 1.333};
  TestModel<DoubleLayerSphericalRefractiveInterface, OpenCVCameraModel>(
      non_svp_params, camera_params);
}

BOOST_AUTO_TEST_CASE(TestDoubleLayerSphericalRefractiveInterfaceCase2) {
  std::vector<double> camera_params = {3200.484,
                                       3200.917,
                                       2790.172,
                                       2108.726,
                                       -0.233072717236466,
                                       0.065022474710061,
                                       1.008118149931866e-06,
                                       -2.863880315774651e-05};
  std::vector<double> non_svp_params = {0.0342007, 0.0366894, 0.0083927, 0.1,
                                        0.007,     1.003,     1.523,     1.333};
  TestModel<DoubleLayerSphericalRefractiveInterface, OpenCVCameraModel>(
      non_svp_params, camera_params);
}

BOOST_AUTO_TEST_CASE(TestDoubleLayerSphericalRefractiveInterfaceCase3) {
  std::vector<double> camera_params = {3200.484,
                                       3200.917,
                                       2790.172,
                                       2108.726,
                                       -0.233072717236466,
                                       0.065022474710061,
                                       1.008118149931866e-06,
                                       -2.863880315774651e-05};
  std::vector<double> non_svp_params = {0.0,   0.0,   0.0,   0.1,
                                        0.007, 1.003, 1.523, 1.333};
  TestModel<DoubleLayerSphericalRefractiveInterface, OpenCVCameraModel>(
      non_svp_params, camera_params);
}

BOOST_AUTO_TEST_CASE(DoubleLayerPlanarRefractiveInterfaceCase1) {
  std::vector<double> camera_params = {3200.484,
                                       3200.917,
                                       2790.172,
                                       2108.726,
                                       -0.233072717236466,
                                       0.065022474710061,
                                       1.008118149931866e-06,
                                       -2.863880315774651e-05};
  Eigen::Vector3d int_normal(RandomReal(-0.1, 0.1), RandomReal(-0.1, 0.1),
                             RandomReal(-0.9, -1.1));
  int_normal.normalize();

  std::vector<double> non_svp_params = {
      int_normal(0), int_normal(1), int_normal(2), 0.05,
      0.007,         1.003,         1.473,         1.333};
  TestModel<DoubleLayerPlanarRefractiveInterface, OpenCVCameraModel>(
      non_svp_params, camera_params);
}
