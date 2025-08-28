#include "calibmar/calibrators/stereo_calibration.h"

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include <colmap/math/math.h>
#include <colmap/math/random.h>

#include "utils/test_helpers.h"

using namespace calibmar;

namespace {
  std::pair<int, int> image_size{2048, 1536};

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

  void GenerateImagePoints(colmap::Camera& camera, const colmap::Rigid3d& relative_pose, const colmap::Rigid3d& absolute_pose,
                           const std::vector<Eigen::Vector3d>& object_points, std::vector<Eigen::Vector2d>& image_points,
                           double std_err) {
    std::default_random_engine generator;
    std::normal_distribution<double> error_dist(0, std_err);

    for (auto& point3D : object_points) {
      Eigen::Vector3d local_point = relative_pose * (absolute_pose * point3D);
      Eigen::Vector2d image_point = camera.ImgFromCam(local_point.hnormalized());
      // add some noise
      Eigen::Vector2d noise_dir(error_dist(generator), error_dist(generator));
      noise_dir.normalize();
      image_point += noise_dir * error_dist(generator);

      image_points.push_back(image_point);
    }
  }

  struct CameraTestData {
    colmap::CameraModelId model_id;
    std::vector<double> params;
  };

  std::vector<CameraTestData> stereo_camera_models = {{colmap::SimplePinholeCameraModel::model_id, {1300, 600, 500}},
                                                      {colmap::PinholeCameraModel::model_id, {1300, 1250, 600, 500}},
                                                      {colmap::SimpleRadialCameraModel::model_id, {850, 600, 500, 0.2}}};

  void SetupData(colmap::Camera& camera1, colmap::Camera& camera2, std::vector<std::vector<Eigen::Vector2d>>& image_points1,
                 std::vector<std::vector<Eigen::Vector2d>>& image_points2,
                 std::vector<std::vector<Eigen::Vector3d>>& object_points1,
                 std::vector<std::vector<Eigen::Vector3d>>& object_points2, const colmap::Rigid3d& relative_pose,
                 const CameraTestData& model) {
    camera1.model_id = model.model_id;
    camera2.model_id = model.model_id;
    camera1.params = model.params;
    camera2.params = model.params;
    camera1.width = camera1.PrincipalPointX() * 2;
    camera2.width = camera2.PrincipalPointX() * 2;
    camera1.height = camera1.PrincipalPointY() * 2;
    camera2.height = camera2.PrincipalPointY() * 2;

    std::vector<Eigen::Vector3d> points3D = CreatePoints3D(15, 15, 0.02);

    size_t n = test_poses.size();
    image_points1.resize(n);
    image_points2.resize(n);
    object_points1.resize(n);
    object_points2.resize(n);
    for (size_t i = 0; i < n; i++) {
      auto& test_pose = test_poses[i];
      Eigen::Quaterniond rot(test_pose[3], test_pose[0], test_pose[1], test_pose[2]);
      Eigen::Vector3d trans(test_pose[4], test_pose[5], test_pose[6]);
      colmap::Rigid3d pose(rot.normalized(), trans);
      GenerateImagePoints(camera1, colmap::Rigid3d(), pose, points3D, image_points1[i], 0);
      GenerateImagePoints(camera2, relative_pose, pose, points3D, image_points2[i], 0);
      object_points1[i] = points3D;
      object_points2[i] = points3D;
    }
  }
}

BOOST_TEST_DONT_PRINT_LOG_VALUE(CameraTestData)
BOOST_DATA_TEST_CASE(StereoCalibration_FullCalibration, boost::unit_test::data::make(stereo_camera_models), model) {
  // This test is somewhat unrealistic, since it will project points out of the images
  colmap::Camera camera1;
  colmap::Camera camera2;
  colmap::Rigid3d relative_pose(Eigen::Quaterniond(Eigen::AngleAxisd(5.0 * (M_PI / 180), Eigen::Vector3d::UnitY())), {0.5, 0, 0});
  colmap::Camera calibration_camera1;
  colmap::Camera calibration_camera2;
  std::vector<std::vector<Eigen::Vector2d>> image_points1;
  std::vector<std::vector<Eigen::Vector2d>> image_points2;
  std::vector<std::vector<Eigen::Vector3d>> object_points;
  colmap::Rigid3d estimated_pose;
  std::vector<colmap::Rigid3d> absolute_poses;
  SetupData(camera1, camera2, image_points1, image_points2, object_points, object_points, relative_pose, model);
  calibration_camera1.model_id = camera1.model_id;
  calibration_camera1.width = camera1.width;
  calibration_camera1.height = camera1.height;
  calibration_camera2.model_id = camera2.model_id;
  calibration_camera2.width = camera2.width;
  calibration_camera2.height = camera2.height;

  stereo_calibration::CalibrateStereoCameras(object_points, object_points, image_points1, image_points2, calibration_camera1,
                                             calibration_camera2, false, false, estimated_pose, absolute_poses);

  // Zero noise should be very close to GT
  BOOST_TEST(ElementWiseClose(calibration_camera1.params, camera1.params, 0.0001));
  BOOST_TEST(ElementWiseClose(calibration_camera2.params, camera2.params, 0.0001));
  BOOST_TEST(ElementWiseClose(estimated_pose.ToMatrix(), relative_pose.ToMatrix(), 0.0001));
}

BOOST_DATA_TEST_CASE(StereoCalibration_PoseOnly, boost::unit_test::data::make(stereo_camera_models), model) {
  colmap::Camera camera1;
  colmap::Camera camera2;
  colmap::Rigid3d relative_pose(Eigen::Quaterniond(Eigen::AngleAxisd(-5 * (M_PI / 180), Eigen::Vector3d::UnitY())), {0.5, 0, 0});
  std::vector<std::vector<Eigen::Vector2d>> image_points1;
  std::vector<std::vector<Eigen::Vector2d>> image_points2;
  std::vector<std::vector<Eigen::Vector3d>> object_points;
  colmap::Rigid3d estimated_pose;
  std::vector<colmap::Rigid3d> absolute_poses;

  SetupData(camera1, camera2, image_points1, image_points2, object_points, object_points, relative_pose, model);

  stereo_calibration::CalibrateStereoCameras(object_points, object_points, image_points1, image_points2, camera1, camera2, true,
                                             true, estimated_pose, absolute_poses);

  BOOST_TEST(ElementWiseClose(estimated_pose.ToMatrix(), relative_pose.ToMatrix(), 0.001));
}

std::vector<CameraTestData> std_dev_models = {
    {colmap::SimplePinholeCameraModel::model_id, {1300, 600, 500}},
    {colmap::PinholeCameraModel::model_id, {1300, 1300, 600, 500}},
    {colmap::SimpleRadialCameraModel::model_id, {1300, 600, 500, 0}},
    {colmap::RadialCameraModel::model_id, {1300, 600, 500, 0, 0}},
    {colmap::OpenCVCameraModel::model_id, {1300, 1300, 600, 500, 0, 0, 0, 0}},
    {colmap::FullOpenCVCameraModel::model_id, {1300, 1300, 600, 500, 0, 0, 0, 0, 0, 0, 0, 0}},
    {colmap::OpenCVFisheyeCameraModel::model_id, {1300, 1300, 600, 500, 0, 0, 0, 0}},
};
BOOST_DATA_TEST_CASE(IntrinsicsDeviationMatchesParamsLength_Stereo, boost::unit_test::data::make(std_dev_models), model) {
  colmap::Camera camera1;
  colmap::Camera camera2;
  colmap::Rigid3d relative_pose(Eigen::Quaterniond(Eigen::AngleAxisd(5.0 * (M_PI / 180), Eigen::Vector3d::UnitY())), {0.5, 0, 0});
  colmap::Camera calibration_camera1;
  colmap::Camera calibration_camera2;
  std::vector<std::vector<Eigen::Vector2d>> image_points1;
  std::vector<std::vector<Eigen::Vector2d>> image_points2;
  std::vector<std::vector<Eigen::Vector3d>> object_points;
  colmap::Rigid3d estimated_pose;
  std::vector<colmap::Rigid3d> absolute_poses;
  SetupData(camera1, camera2, image_points1, image_points2, object_points, object_points, relative_pose, model);
  calibration_camera1.model_id = camera1.model_id;
  calibration_camera1.width = camera1.width;
  calibration_camera1.height = camera1.height;
  calibration_camera2.model_id = camera2.model_id;
  calibration_camera2.width = camera2.width;
  calibration_camera2.height = camera2.height;

  std::vector<double> intrinsics1, intrinsics2, extrinsics1, extrinsics2;
  stereo_calibration::StereoStdDeviations std_devs;
  std_devs.std_deviations_extrinsics1 = &extrinsics1;
  std_devs.std_deviations_extrinsics2 = &extrinsics2;
  std_devs.std_deviations_intrinsics1 = &intrinsics1;
  std_devs.std_deviations_intrinsics2 = &intrinsics2;

  stereo_calibration::CalibrateStereoCameras(object_points, object_points, image_points1, image_points2, calibration_camera1,
                                             calibration_camera2, false, false, estimated_pose, absolute_poses, &std_devs);

  int intrinsics_size = model.params.size();
  int extrinsics_size = 7;

  BOOST_TEST(std_devs.std_deviations_extrinsics1->size() == extrinsics_size);
  BOOST_TEST(std_devs.std_deviations_extrinsics2->size() == extrinsics_size);
  BOOST_TEST(std_devs.std_deviations_intrinsics1->size() == intrinsics_size);
  BOOST_TEST(std_devs.std_deviations_intrinsics2->size() == intrinsics_size);
}

BOOST_DATA_TEST_CASE(StereoCalibration_AsymetricPoints, boost::unit_test::data::make(stereo_camera_models), model) {
  // This test is somewhat unrealistic, since it will project points out of the images
  colmap::Camera camera1;
  colmap::Camera camera2;
  colmap::Rigid3d relative_pose(Eigen::Quaterniond(Eigen::AngleAxisd(5.0 * (M_PI / 180), Eigen::Vector3d::UnitY())), {0.5, 0, 0});
  colmap::Camera calibration_camera1;
  colmap::Camera calibration_camera2;
  std::vector<std::vector<Eigen::Vector2d>> image_points1;
  std::vector<std::vector<Eigen::Vector2d>> image_points2;
  std::vector<std::vector<Eigen::Vector3d>> object_points1;
  std::vector<std::vector<Eigen::Vector3d>> object_points2;
  colmap::Rigid3d estimated_pose;
  std::vector<colmap::Rigid3d> absolute_poses;
  SetupData(camera1, camera2, image_points1, image_points2, object_points1, object_points2, relative_pose, model);
  calibration_camera1.model_id = camera1.model_id;
  calibration_camera1.width = camera1.width;
  calibration_camera1.height = camera1.height;
  calibration_camera2.model_id = camera2.model_id;
  calibration_camera2.width = camera2.width;
  calibration_camera2.height = camera2.height;

  for (int i = 0; i < image_points2.size(); i++) {
    for (size_t i = 0; i < 10; i++) {
      // Remove some points only from one camera
      image_points2[i].pop_back();
      object_points2[i].pop_back();
    }
  }

  stereo_calibration::CalibrateStereoCameras(object_points1, object_points2, image_points1, image_points2, calibration_camera1,
                                             calibration_camera2, false, false, estimated_pose, absolute_poses);

  // Zero noise should be very close to GT
  BOOST_TEST(ElementWiseClose(calibration_camera1.params, camera1.params, 0.0001));
  BOOST_TEST(ElementWiseClose(calibration_camera2.params, camera2.params, 0.0001));
  BOOST_TEST(ElementWiseClose(estimated_pose.ToMatrix(), relative_pose.ToMatrix(), 0.0001));
}