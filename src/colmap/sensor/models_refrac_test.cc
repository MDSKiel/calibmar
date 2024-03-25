#include "colmap/sensor/models_refrac.h"

#include "colmap/math/random.h"

#include <gtest/gtest.h>

namespace colmap {

template <typename CameraRefracModel, typename CameraModel>
void TestCamToCamFromImg(const std::vector<double>& cam_params,
                         const std::vector<double>& refrac_params,
                         const double u0,
                         const double v0,
                         const double w0) {
  double x, y;
  Eigen::Vector3d uvw;
  double d = sqrt(u0 * u0 + v0 * v0 + w0 * w0);
  CameraRefracModel::template ImgFromCam<CameraModel, double>(
      cam_params.data(), refrac_params.data(), u0, v0, w0, &x, &y);
  const Eigen::Vector2d xy =
      CameraRefracModelImgFromCam(CameraModel::model_id,
                                  CameraRefracModel::refrac_model_id,
                                  cam_params,
                                  refrac_params,
                                  Eigen::Vector3d(u0, v0, w0));
  EXPECT_EQ(x, xy.x());
  EXPECT_EQ(y, xy.y());
  CameraRefracModel::template CamFromImgPoint<CameraModel, double>(
      cam_params.data(), refrac_params.data(), x, y, d, &uvw);
  EXPECT_NEAR(uvw.x(), u0, 1e-6);
  EXPECT_NEAR(uvw.y(), v0, 1e-6);
  EXPECT_NEAR(uvw.z(), w0, 1e-6);
}

template <typename CameraRefracModel, typename CameraModel>
void TestCamFromImgToImg(const std::vector<double>& cam_params,
                         const std::vector<double>& refrac_params,
                         const double x0,
                         const double y0,
                         const double d0) {
  double x, y;
  Eigen::Vector3d uvw;
  CameraRefracModel::template CamFromImgPoint<CameraModel, double>(
      cam_params.data(), refrac_params.data(), x0, y0, d0, &uvw);
  const Eigen::Vector3d uvw2 =
      CameraRefracModelCamFromImgPoint(CameraModel::model_id,
                                       CameraRefracModel::refrac_model_id,
                                       cam_params,
                                       refrac_params,
                                       Eigen::Vector2d(x0, y0),
                                       d0);
  EXPECT_EQ(uvw.x(), uvw2.x());
  EXPECT_EQ(uvw.y(), uvw2.y());
  EXPECT_EQ(uvw.z(), uvw2.z());
  CameraRefracModel::template ImgFromCam<CameraModel, double>(
      cam_params.data(),
      refrac_params.data(),
      uvw.x(),
      uvw.y(),
      uvw.z(),
      &x,
      &y);
  EXPECT_NEAR(x, x0, 1e-6) << "Expect x to be " << x0 << ", but " << x
                           << " computed";
  EXPECT_NEAR(y, y0, 1e-6) << "Expect y to be " << y0 << ", but " << y
                           << " computed";
}

template <typename CameraRefracModel, typename CameraModel>
void TestModel(const std::vector<double>& cam_params,
               const std::vector<double>& refrac_params) {
  EXPECT_TRUE(CameraRefracModelVerifyParams(CameraRefracModel::refrac_model_id,
                                            refrac_params));

  EXPECT_EQ(CameraRefracModelParamsInfo(CameraRefracModel::refrac_model_id),
            CameraRefracModel::refrac_params_info);
  EXPECT_EQ(CameraRefracModelNumParams(CameraRefracModel::refrac_model_id),
            CameraRefracModel::num_params);

  EXPECT_TRUE(
      ExistsCameraRefracModelWithName(CameraRefracModel::refrac_model_name));
  EXPECT_FALSE(ExistsCameraRefracModelWithName(
      CameraRefracModel::refrac_model_name + "FOO"));

  EXPECT_TRUE(
      ExistsCameraRefracModelWithId(CameraRefracModel::refrac_model_id));
  EXPECT_FALSE(ExistsCameraRefracModelWithId(
      static_cast<CameraRefracModelId>(123456789)));

  EXPECT_EQ(CameraRefracModelNameToId(
                CameraRefracModelIdToName(CameraRefracModel::refrac_model_id)),
            CameraRefracModel::refrac_model_id);
  EXPECT_EQ(CameraRefracModelIdToName(CameraRefracModelNameToId(
                CameraRefracModel::refrac_model_name)),
            CameraRefracModel::refrac_model_name);
  EXPECT_EQ(&CameraRefracModelOptimizableParamsIdxs(
                CameraRefracModel::refrac_model_id),
            &CameraRefracModel::optimizable_params_idxs);
  Eigen::Vector3d refrac_axis(
      refrac_params[0], refrac_params[1], refrac_params[2]);
  refrac_axis.normalize();
  EXPECT_EQ(CameraRefracModelRefractionAxis(CameraRefracModel::refrac_model_id,
                                            refrac_params),
            refrac_axis);

  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter)
  for (double u = -0.5; u <= 0.5; u += 0.1) {
    // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter)
    for (double v = -0.5; v <= 0.5; v += 0.1) {
      // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter)
      for (double w = 0.5; w <= 5.5; w += 0.2) {
        TestCamToCamFromImg<CameraRefracModel, CameraModel>(
            cam_params, refrac_params, u, v, w);
      }
    }
  }

  for (double x = 0; x <= 5568.0; x += 50.0) {
    for (double y = 0; y <= 4176.0; y += 50.0) {
      for (int n = 0; n < 10; n++) {
        const double d = RandomUniformReal(0.2, 10.0);
        TestCamFromImgToImg<CameraRefracModel, CameraModel>(
            cam_params, refrac_params, x, y, d);
      }
    }
  }
}

TEST(FlatPort, Nominal) {
  std::vector<double> cam_params = {3200.484,
                                    3200.917,
                                    2790.172,
                                    2108.726,
                                    -0.233072717236466,
                                    0.065022474710061,
                                    1.008118149931866e-06,
                                    -2.863880315774651e-05};
  Eigen::Vector3d int_normal(RandomUniformReal(-0.1, 0.1),
                             RandomUniformReal(-0.1, 0.1),
                             RandomUniformReal(0.9, 1.1));
  int_normal.normalize();

  std::vector<double> refrac_params = {int_normal(0),
                                       int_normal(1),
                                       int_normal(2),
                                       0.05,
                                       0.007,
                                       1.003,
                                       1.473,
                                       1.333};
  TestModel<FlatPort, OpenCVCameraModel>(cam_params, refrac_params);
}

TEST(DomePort, Case1) {
  std::vector<double> cam_params = {3200.484,
                                    3200.917,
                                    2790.172,
                                    2108.726,
                                    -0.233072717236466,
                                    0.065022474710061,
                                    1.008118149931866e-06,
                                    -2.863880315774651e-05};
  std::vector<double> refrac_params = {
      0.00042007, 0.00366894, 0.0283927, 0.05, 0.007, 1.003, 1.473, 1.333};
  TestModel<DomePort, OpenCVCameraModel>(cam_params, refrac_params);
}

TEST(DomePort, Case2) {
  std::vector<double> cam_params = {3200.484,
                                    3200.917,
                                    2790.172,
                                    2108.726,
                                    -0.233072717236466,
                                    0.065022474710061,
                                    1.008118149931866e-06,
                                    -2.863880315774651e-05};
  std::vector<double> refrac_params = {
      0.0342007, 0.0366894, 0.0083927, 0.1, 0.007, 1.003, 1.523, 1.333};
  TestModel<DomePort, OpenCVCameraModel>(cam_params, refrac_params);
}

TEST(DomePort, Case3) {
  std::vector<double> cam_params = {3200.484,
                                    3200.917,
                                    2790.172,
                                    2108.726,
                                    -0.233072717236466,
                                    0.065022474710061,
                                    1.008118149931866e-06,
                                    -2.863880315774651e-05};
  std::vector<double> refrac_params = {
      0.0, 0.0, 0.0, 0.1, 0.007, 1.003, 1.523, 1.333};
  TestModel<DomePort, OpenCVCameraModel>(cam_params, refrac_params);
}

}  // namespace colmap