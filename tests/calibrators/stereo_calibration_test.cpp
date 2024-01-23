#include "calibmar/calibrators/stereo_calibrator.h"

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

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

  void PrepareCalibration(Calibration& calibration, std::pair<int, int> image_size, CameraModelType camera_model,
                          std::vector<double> camera_params, colmap::Rigid3d relative_pose) {
    std::default_random_engine generator;
    std::normal_distribution<double> error_dist(0, 0.125);

    colmap::Camera camera;
    camera.SetWidth(image_size.first);
    camera.SetHeight(image_size.second);
    camera.SetModelIdFromName(calibmar::CameraModel::CameraModels().at(camera_model).model_name);
    camera.SetParams(camera_params);

    std::vector<Eigen::Vector3d> points3D = CreatePoints3D(10, 11, 0.02);

    for (auto& point3D : points3D) {
      calibration.AddPoint3D(point3D);
    }
    // for each test pose artificially create a image point to calibrate from
    for (auto& test_pose : test_poses) {
      Eigen::Quaterniond rot(test_pose[3], test_pose[0], test_pose[1], test_pose[2]);
      Eigen::Vector3d trans(test_pose[4], test_pose[5], test_pose[6]);
      rot.normalize();
      colmap::Rigid3d pose(rot, trans);

      std::unordered_map<int, int> correspondences;
      std::vector<Eigen::Vector2d> points2D;
      for (auto& calibrationPoint3D : calibration.Points3D()) {
        Eigen::Vector3d& point3D = calibrationPoint3D.second;

        Eigen::Vector3d local_point = relative_pose * (pose * point3D);
        Eigen::Vector2d image_point = camera.ImgFromCam(local_point.hnormalized());
        // add some noise
        Eigen::Vector2d noise_dir(error_dist(generator), error_dist(generator));
        noise_dir.normalize();
        image_point += noise_dir * error_dist(generator);

        points2D.push_back(image_point);
        correspondences[calibrationPoint3D.first] = points2D.size() - 1;
      }

      Image image;
      image.SetPoints2D(points2D);
      for (auto& correspondence : correspondences) {
        image.SetPoint3DforPoint2D(correspondence.first, correspondence.second);
      }
      calibration.AddImage(image);
    }
  }
}

BOOST_AUTO_TEST_CASE(BasicStereoCalibration) {
  Calibration calibration1;
  Calibration calibration2;
  StereoCalibrator::Options options;

  options.camera_model = CameraModelType::SimplePinholeCameraModel;
  options.image_size = {1280, 1024};
  std::vector<double> expected_params = {1372.5, 1280 / 2.0, 1024 / 2.0};
  colmap::Rigid3d relative_pose(Eigen::Quaterniond(Eigen::AngleAxisd(-5 * (M_PI / 180), Eigen::Vector3d::UnitY())), {0.5, 0, 0});
  PrepareCalibration(calibration1, options.image_size, options.camera_model, expected_params, colmap::Rigid3d{});
  PrepareCalibration(calibration2, options.image_size, options.camera_model, expected_params, colmap::Inverse(relative_pose));
  StereoCalibrator calibrator(options);

  calibrator.Calibrate(calibration1, calibration2);

  std::vector<double> actual_params1 = calibration1.Camera().Params();
  std::vector<double> actual_params2 = calibration2.Camera().Params();
  colmap::Rigid3d actual_relative_pose = calibration2.CameraToWorldStereo().value();
  double tolerance = 0.3;
  BOOST_TEST(abs(actual_params1[0] - expected_params[0]) < tolerance);
  BOOST_TEST(abs(actual_params1[1] - expected_params[1]) < tolerance);
  BOOST_TEST(abs(actual_params1[2] - expected_params[2]) < tolerance);
  BOOST_TEST(abs(actual_params2[0] - expected_params[0]) < tolerance);
  BOOST_TEST(abs(actual_params2[1] - expected_params[1]) < tolerance);
  BOOST_TEST(abs(actual_params2[2] - expected_params[2]) < tolerance);
  BOOST_TEST(ElementWiseClose(relative_pose.ToMatrix(), actual_relative_pose.ToMatrix(), 0.01));
}