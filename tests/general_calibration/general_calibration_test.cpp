#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "calibmar/calibrators/general_calibration.h"
#include "calibmar/calibrators/opencv_calibration.h"
#include "colmap/estimators/homography_matrix.h"
#include "colmap/geometry/homography_matrix.h"
#include "colmap/math/random.h"
#include "colmap/scene/camera.h"
#include "colmap/sensor/models.h"

#include "utils/test_helpers.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

using namespace calibmar;

namespace {
  // in format  {quat, trans} => {x, y, z, w, x, y, z}
  std::vector<std::vector<double>> test_poses = {{-0.000496536, 0.252943, 2.45128e-05, 0.967481, 0.287759, -0.289604, 1.0959},
                                                 {0.0420354, 0.253093, -0.0107101, 0.966469, 0.25171, -0.288986, 1.06792},
                                                 {0.130199, 0.242548, -0.0326248, 0.960809, 0.172041, -0.282769, 1.00455},
                                                 {0.220024, 0.215725, -0.0498013, 0.950038, 0.0936999, -0.268469, 0.93915},
                                                 {0.262571, 0.170146, -0.0469419, 0.948632, 0.060451, -0.25027, 0.902891},
                                                 {0.229598, 0.0605705, -0.0338772, 0.970808, 0.0656431, -0.192304, 0.890473},
                                                 {0.151242, -0.117097, -0.0481633, 0.980354, 0.0712398, -0.0851783, 0.877166},
                                                 {0.0658327, -0.289528, -0.0994595, 0.949709, 0.0734617, 0.0217881, 0.86452},
                                                 {-0.0162237, -0.370666, -0.152047, 0.916093, 0.0687984, 0.0723096, 0.864322},
                                                 {-0.102117, -0.296687, -0.164624, 0.935119, 0.0560959, 0.0693572, 0.860182},
                                                 {-0.177525, -0.116679, -0.130025, 0.968486, 0.0606593, 0.0563274, 0.84197},
                                                 {-0.217524, 0.0731064, -0.0657052, 0.971093, 0.0956975, 0.0341515, 0.81907},
                                                 {-0.222058, 0.165276, -0.00967184, 0.960875, 0.138258, 0.00463443, 0.805565},
                                                 {-0.167806, 0.17489, 0.0253441, 0.969852, 0.196652, -0.032798, 0.884306},
                                                 {-0.0548886, 0.174335, 0.0509519, 0.981834, 0.278361, -0.0697696, 1.05191},
                                                 {0.0517771, 0.159238, 0.0684082, 0.983505, 0.351207, -0.0932214, 1.21631},
                                                 {0.101385, 0.151714, 0.074391, 0.980393, 0.382891, -0.100532, 1.2907}};

  void SetupPoints(colmap::Camera& camera, std::vector<Eigen::Vector3d>& points3D,
                   std::vector<std::vector<Eigen::Vector3d>>& object_points,
                   std::vector<std::vector<Eigen::Vector2d>>& image_points, std::vector<colmap::Rigid3d>& poses,
                   int number_of_poses, double std_dev_pixel_error) {
    colmap::SetPRNGSeed(-1);
    object_points.resize(number_of_poses);
    image_points.resize(number_of_poses);

    for (size_t i = 0; i < number_of_poses; i++) {
      std::vector<Eigen::Vector2d>& points2D = image_points[i];
      points2D.reserve(points3D.size());

      auto& pose_data = test_poses[i];
      colmap::Rigid3d pose(Eigen::Quaterniond(pose_data[3], pose_data[0], pose_data[1], pose_data[2]),
                           Eigen::Vector3d(pose_data[4], pose_data[5], pose_data[6]));

      poses.push_back(pose);

      for (const auto& point3D : points3D) {
        Eigen::Vector3d local_point = pose * point3D;
        Eigen::Vector2d image_point = camera.ImgFromCam(local_point.hnormalized());
        image_point.x() += colmap::RandomGaussian(0.0, std_dev_pixel_error);
        image_point.y() += colmap::RandomGaussian(0.0, std_dev_pixel_error);
        points2D.push_back(image_point);
      }

      object_points[i] = points3D;
    }
  }
}

BOOST_AUTO_TEST_CASE(EstimateKFromHomographies) {
  // generate 2D points
  colmap::Camera camera;
  double fx_true = 800;
  double fy_true = fx_true;
  double cx_true = 300;
  double cy_true = 400;
  camera.InitializeWithId(colmap::SimplePinholeCameraModel::kModelId, fx_true, cx_true * 2, cy_true * 2);

  std::vector<Eigen::Vector3d> points3D = CreatePoints3D(10, 10, 0.01);

  // use minimal number of views
  int n = 2;
  std::vector<std::vector<Eigen::Vector2d>> point_sets;
  std::vector<std::vector<Eigen::Vector3d>> point_sets_3D;
  std::vector<colmap::Rigid3d> poses;
  SetupPoints(camera, points3D, point_sets_3D, point_sets, poses, n, 0);

  // create 2D plane points
  std::vector<std::vector<Eigen::Vector2d>> object_plane_points(point_sets_3D.size());
  for (size_t i = 0; i < point_sets_3D.size(); i++) {
    object_plane_points[i].reserve(point_sets_3D[i].size());
    for (size_t j = 0; j < point_sets_3D[i].size(); j++) {
      const auto& point_obj = point_sets_3D[i][j];
      object_plane_points[i].push_back({point_obj.x(), point_obj.y()});
    }
  }

  double fx, fy, cx, cy;
  general_calibration::EstimateKFromHomographies(object_plane_points, point_sets, fx, fy, cx, cy);

  BOOST_ASSERT(abs(fx - fx_true) < 0.0005);
  BOOST_ASSERT(abs(fy - fy_true) < 0.0005);
  BOOST_ASSERT(abs(cx - cx_true) < 0.0005);
  BOOST_ASSERT(abs(cy - cy_true) < 0.0005);
}

BOOST_AUTO_TEST_CASE(PoseFromHomography) {
  // generate 2D points
  colmap::Camera camera;
  double fx_true = 800;
  double fy_true = fx_true;
  double cx_true = 300;
  double cy_true = 400;
  camera.InitializeWithId(colmap::SimplePinholeCameraModel::kModelId, fx_true, cx_true * 2, cy_true * 2);

  std::vector<Eigen::Vector3d> points3D = CreatePoints3D(10, 10, 0.01);

  int n = 1;
  std::vector<std::vector<Eigen::Vector2d>> point_sets;
  std::vector<std::vector<Eigen::Vector3d>> point_sets_3D;
  std::vector<colmap::Rigid3d> poses;
  SetupPoints(camera, points3D, point_sets_3D, point_sets, poses, n, 0);

  // create 2D plane points
  std::vector<std::vector<Eigen::Vector2d>> object_plane_points(point_sets_3D.size());
  for (size_t i = 0; i < point_sets_3D.size(); i++) {
    object_plane_points[i].reserve(point_sets_3D[i].size());
    for (size_t j = 0; j < point_sets_3D[i].size(); j++) {
      const auto& point_obj = point_sets_3D[i][j];
      object_plane_points[i].push_back({point_obj.x(), point_obj.y()});
    }
  }

  colmap::Rigid3d pose = poses[0];
  Eigen::Matrix3d H = colmap::HomographyMatrixEstimator::Estimate(object_plane_points[0], point_sets[0])[0];

  Eigen::Matrix3d K = camera.CalibrationMatrix();
  colmap::Rigid3d estimated;
  general_calibration::EstimatePoseFromHomography(H, K, estimated);

  BOOST_ASSERT(ElementWiseClose(pose.ToMatrix(), estimated.ToMatrix(), 0.00001));
}

struct CameraTestData {
  int model_id;
  std::vector<double> params;
};

BOOST_TEST_DONT_PRINT_LOG_VALUE(CameraTestData)
std::vector<CameraTestData> camera_models = {
    {colmap::SimplePinholeCameraModel::kModelId, {1300, 600, 500}},
    {colmap::PinholeCameraModel::kModelId, {1300, 1250, 600, 500}},
    {colmap::SimpleRadialCameraModel::kModelId, {850, 600, 500, 0.8}},
    {colmap::RadialCameraModel::kModelId, {1000, 600, 500, -0.1, -0.05}},
    {colmap::OpenCVCameraModel::kModelId, {900, 900, 600, 500, 0.01, -0.08, 0.02, -0.005}},
    {colmap::FullOpenCVCameraModel::kModelId, {1200, 1200, 600, 500, 0.1, 0.08, 0.02, -0.005, 0.1, -0.02, -0.03, -0.003}},
    {colmap::OpenCVFisheyeCameraModel::kModelId, {1200, 1200, 600, 500, -0.1, -0.2, -0.4, -0.8}}};
BOOST_DATA_TEST_CASE(CalibrateCamera, boost::unit_test::data::make(camera_models), model) {
  colmap::Camera camera;
  camera.SetModelId(model.model_id);
  camera.SetWidth(1200);
  camera.SetHeight(1000);
  camera.SetParams(model.params);
  BOOST_ASSERT(camera.VerifyParams());
  std::vector<Eigen::Vector3d> points3D = CreatePoints3D(15, 15, 0.02);
  std::vector<std::vector<Eigen::Vector3d>> object_points;
  std::vector<std::vector<Eigen::Vector2d>> image_points;
  std::vector<colmap::Rigid3d> poses;

  // without error we expect basically perfect calibration
  SetupPoints(camera, points3D, object_points, image_points, poses, test_poses.size(), 0);

  // remove views with points outside of the image
  auto object_it = object_points.begin();
  for (auto image_it = image_points.begin(); image_it != image_points.end();) {
    bool erase = false;
    for (const auto& point : *image_it) {
      if (point.x() < 0 || point.x() >= camera.Width() || point.y() < 0 || point.y() >= camera.Height()) {
        erase = true;
        break;
      }
    }

    if (erase) {
      image_it = image_points.erase(image_it);
      object_it = object_points.erase(object_it);
    }
    else {
      image_it++;
      object_it++;
    }
  }

  if (image_points.size() < test_poses.size() / 2) {
    BOOST_FAIL("More than half of views contain points outside of the image, for model: " +
               colmap::CameraModelIdToName(model.model_id));
  }

  colmap::Camera calibration_camera;
  calibration_camera.SetModelId(model.model_id);
  calibration_camera.SetWidth(camera.Width());
  calibration_camera.SetHeight(camera.Height());
  std::vector<colmap::Rigid3d> poses2;
  general_calibration::CalibrateCamera(object_points, image_points, calibration_camera, false, poses2);

  // For FullOpenCVCameraModel the poses might be too incomplete (they could be bad in general)
  double tolerance = model.model_id != colmap::FullOpenCVCameraModel::model_id ? 0.001 : 0.1;

  ASSERT_WITH_MSG(ElementWiseClose(camera.Params(), calibration_camera.Params(), tolerance),
                  "Camera calibration out of tolerance for model: " + colmap::CameraModelIdToName(model.model_id));
}