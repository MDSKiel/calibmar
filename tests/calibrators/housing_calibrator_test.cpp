#include "calibmar/calibrators/housing_calibrator.h"

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "utils/test_helpers.h"

using namespace calibmar;

namespace {
  std::pair<int, int> image_size{2048, 1536};

  void PrepareCalibrationDomePort(Calibration& calibration);
  void PrepareCalibrationFlatPort(Calibration& calibration, std::pair<int, int> image_size, CameraModelType camera_model,
                                  HousingInterfaceType housing_interface, std::vector<double> camera_params,
                                  std::vector<double> housing_params);
}

BOOST_AUTO_TEST_CASE(BasicHousingCalibrationFlat) {
  Calibration calibration;
  HousingCalibrator::Options options;
  Eigen::Vector3d normal(0.1, 0.1, 1);
  normal.normalize();
  std::vector<double> expected_params = {normal.x(), normal.y(), normal.z(), 0.02, 0.014, 1.003, 1.473, 1.333};
  options.camera_params = {864.91, 640, 512};
  options.initial_housing_params = {0, 0, 1, 0.04, 0.014, 1.003, 1.473, 1.333};  // intentionally wrong distance
  options.camera_model = CameraModelType::SimplePinholeCameraModel;
  options.housing_interface = HousingInterfaceType::DoubleLayerPlanarRefractive;
  options.pattern_cols_rows = {8, 7};
  options.image_size = {1280, 1024};
  HousingCalibrator calibrator(options);
  PrepareCalibrationFlatPort(calibration, options.image_size, options.camera_model, options.housing_interface,
                             options.camera_params, expected_params);

  calibrator.Calibrate(calibration);

  std::vector<double> actual_params = calibration.Camera().refrac_params;
  double angle_offset_interface =
      std::acos(normal.dot(Eigen::Vector3d(actual_params[0], actual_params[1], actual_params[2]))) * (180 / M_PI);
  BOOST_TEST(abs(angle_offset_interface) < 0.04);
  BOOST_TEST(abs(actual_params[3] - expected_params[3]) < 0.01);  // distance is not estimated well
}

BOOST_AUTO_TEST_CASE(BasicHousingCalibrationDome) {
  Calibration calibration;
  PrepareCalibrationDomePort(calibration);
  HousingCalibrator::Options options;
  options.camera_params = {1024, 1024, 1024, 768, 0, 0, 0, 0};
  options.initial_housing_params = {0, 0, 0, 0.0501, 0.007, 1.003, 1.473, 1.333};
  options.camera_model = CameraModelType::OpenCVCameraModel;
  options.housing_interface = HousingInterfaceType::DoubleLayerSphericalRefractive;
  options.pattern_cols_rows = {8, 7};
  options.image_size = image_size;
  options.estimate_initial_dome_offset = true;
  HousingCalibrator calibrator(options);

  calibrator.Calibrate(calibration);

  std::vector<double> params = calibration.Camera().refrac_params;
  std::vector<double> expected_params = {7.3261e-10, 7.49404e-09, -0.03, 0.0501, 0.007, 1.003, 1.473, 1.333};
  double tolerance = 0.001;
  BOOST_TEST(abs(params[0] - expected_params[0]) < tolerance);
  BOOST_TEST(abs(params[1] - expected_params[1]) < tolerance);
  BOOST_TEST(abs(params[2] - expected_params[2]) < tolerance);
}

// A regression test, correct values are not known.
BOOST_AUTO_TEST_CASE(BasicHousingCalibrationCalculatesRmsAndStdDeviations) {
  Calibration calibration;
  PrepareCalibrationDomePort(calibration);
  HousingCalibrator::Options options;
  options.camera_params = {1024, 1024, 1024, 768, 0, 0, 0, 0};
  options.initial_housing_params = {0, 0, 0, 0.0501, 0.007, 1.003, 1.473, 1.333};
  options.camera_model = CameraModelType::OpenCVCameraModel;
  options.housing_interface = HousingInterfaceType::DoubleLayerSphericalRefractive;
  options.pattern_cols_rows = {8, 7};
  options.estimate_initial_dome_offset = true;
  options.image_size = image_size;
  HousingCalibrator calibrator(options);

  calibrator.Calibrate(calibration);

  double rms = calibration.CalibrationRms();
  std::vector<double> std_deviations = calibration.HousingParamsStdDeviations();
  std::vector<double> expected_deviations = {
      0.00063600567604551659, 0.00061898976187972881, 0.0012159999919902228, 0, 0, 0, 0, 0};
  Eigen::Map<Eigen::VectorXd> actual(std_deviations.data(), std_deviations.size());
  Eigen::Map<Eigen::VectorXd> expected(expected_deviations.data(), expected_deviations.size());
  BOOST_TEST(abs(rms - 0.24220783389794076) < 0.00001);
  BOOST_TEST(ElementWiseClose(actual, expected, 0.00001));
}

namespace {
  extern std::vector<Eigen::Vector3d> points3D;
  extern std::vector<std::vector<Eigen::Vector2d>> point2DSetsDome;
  extern std::vector<std::vector<double>> test_poses;

  void PrepareCalibrationDomePort(Calibration& calibration) {
    for (auto& point3D : points3D) {
      calibration.AddPoint3D(point3D * 0.04);
    }

    for (auto& set : point2DSetsDome) {
      Image image;
      image.SetPoints2D(set);

      for (size_t i = 0; i < set.size(); i++) {
        image.SetPoint3DforPoint2D(i, i);
      }

      calibration.AddImage(image);
    }
  }

  void PrepareCalibrationFlatPort(Calibration& calibration, std::pair<int, int> image_size, CameraModelType camera_model,
                                  HousingInterfaceType housing_interface, std::vector<double> camera_params,
                                  std::vector<double> housing_params) {
    std::default_random_engine generator;
    // With this setup (perhaps the poses arent too great?) the reconstruction doesn't tolerate too much pixel-error
    std::normal_distribution<double> error_dist(0, 0.125);

    colmap::Camera camera;
    camera.width = image_size.first;
    camera.height = image_size.second;
    camera.model_id = colmap::CameraModelNameToId(calibmar::CameraModel::CameraModels().at(camera_model).model_name);
    camera.refrac_model_id =
        colmap::CameraRefracModelNameToId(calibmar::HousingInterface::HousingInterfaces().at(housing_interface).model_name);
    camera.params = camera_params;
    camera.refrac_params = housing_params;

    for (auto& point3D : points3D) {
      calibration.AddPoint3D(point3D);
    }
    // for each test pose artificially create a image point to calibrate from
    for (auto& pose : test_poses) {
      Eigen::Quaterniond rot(pose[3], pose[0], pose[1], pose[2]);
      Eigen::Vector3d trans(pose[4], pose[5], pose[6]);
      rot.normalize();

      std::unordered_map<int, int> correspondences;
      std::vector<Eigen::Vector2d> points2D;
      for (auto& calibrationPoint3D : calibration.Points3D()) {
        Eigen::Vector3d& point3D = calibrationPoint3D.second;

        Eigen::Vector3d local_point = rot * point3D + trans;
        Eigen::Vector2d image_point = camera.ImgFromCamRefrac(local_point);
        // add some noise
        Eigen::Vector2d noise_dir(error_dist(generator), error_dist(generator));
        noise_dir.normalize();
        image_point += noise_dir * error_dist(generator);

        if (image_point.x() < 0 || image_point.x() > camera.width || image_point.y() < 0 || image_point.y() > camera.height ||
            std::isnan(image_point.x()) || std::isnan(image_point.y())) {
          continue;  // ignore points which did not project into the image
        }
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

  // in format  {quat, trans} => {x, y, z, w, x, y, z}
  std::vector<std::vector<double>> test_poses = {{
                                                     -0.5673,
                                                     -0.001805,
                                                     0.8235,
                                                     -0.005699,
                                                     -2.82,
                                                     3.054,
                                                     16.32,
                                                 },
                                                 {
                                                     -0.1591,
                                                     0.1589,
                                                     0.689,
                                                     0.689,
                                                     3.22,
                                                     -2.34,
                                                     11.1,
                                                 },
                                                 {
                                                     0.1915,
                                                     0.286,
                                                     0.9381,
                                                     -0.03859,
                                                     2.223,
                                                     1.838,
                                                     9.16,
                                                 },
                                                 {
                                                     0.186,
                                                     0.09415,
                                                     0.978,
                                                     -0.008112,
                                                     7.89,
                                                     1.188,
                                                     19.66,
                                                 },
                                                 {
                                                     0.0001447,
                                                     -8.19e-06,
                                                     1,
                                                     -2.287e-05,
                                                     1.076,
                                                     3.037,
                                                     10.76,
                                                 },
                                                 {
                                                     -0.07786,
                                                     -0.01013,
                                                     0.9905,
                                                     0.1127,
                                                     -1.719,
                                                     -0.3717,
                                                     22.02,
                                                 },
                                                 {
                                                     0.12,
                                                     -0.08275,
                                                     0.9893,
                                                     0.006299,
                                                     4.62,
                                                     3.504,
                                                     12.75,
                                                 },
                                                 {
                                                     -0.02629,
                                                     0.06526,
                                                     0.9975,
                                                     -0.01062,
                                                     0.5062,
                                                     3.065,
                                                     12.41,
                                                 },
                                                 {
                                                     -0.1228,
                                                     0.1225,
                                                     0.6964,
                                                     0.6964,
                                                     2.97,
                                                     -3.191,
                                                     10.86,
                                                 },
                                                 {
                                                     -0.0001644,
                                                     0.0003252,
                                                     1,
                                                     -1.264e-05,
                                                     3.825,
                                                     3.037,
                                                     10.76,
                                                 }};

  std::vector<Eigen::Vector3d> points3D = {
      {1.0, 1.0, 0}, {2.0, 1.0, 0}, {3.0, 1.0, 0}, {4.0, 1.0, 0}, {5.0, 1.0, 0}, {6.0, 1.0, 0}, {1.0, 2.0, 0},
      {2.0, 2.0, 0}, {3.0, 2.0, 0}, {4.0, 2.0, 0}, {5.0, 2.0, 0}, {6.0, 2.0, 0}, {1.0, 3.0, 0}, {2.0, 3.0, 0},
      {3.0, 3.0, 0}, {4.0, 3.0, 0}, {5.0, 3.0, 0}, {6.0, 3.0, 0}, {1.0, 4.0, 0}, {2.0, 4.0, 0}, {3.0, 4.0, 0},
      {4.0, 4.0, 0}, {5.0, 4.0, 0}, {6.0, 4.0, 0}, {1.0, 5.0, 0}, {2.0, 5.0, 0}, {3.0, 5.0, 0}, {4.0, 5.0, 0},
      {5.0, 5.0, 0}, {6.0, 5.0, 0}, {1.0, 6.0, 0}, {2.0, 6.0, 0}, {3.0, 6.0, 0}, {4.0, 6.0, 0}, {5.0, 6.0, 0},
      {6.0, 6.0, 0}, {1.0, 7.0, 0}, {2.0, 7.0, 0}, {3.0, 7.0, 0}, {4.0, 7.0, 0}, {5.0, 7.0, 0}, {6.0, 7.0, 0}};

  std::vector<std::vector<Eigen::Vector2d>> point2DSetsDome = {
      {
          {916.147, 834.654}, {846.94, 850.674},  {775.302, 867.357}, {700.657, 884.635}, {622.947, 902.791}, {541.586, 921.696},
          {899.964, 767.07},  {830.986, 781.925}, {759.519, 797.272}, {685.082, 813.038}, {607.587, 829.625}, {526.556, 846.939},
          {883.792, 700.376}, {814.99, 713.98},   {743.633, 727.997}, {669.488, 742.528}, {592.208, 757.573}, {511.448, 773.3},
          {867.792, 634.216}, {799.056, 646.676}, {727.732, 659.512}, {653.631, 672.686}, {576.427, 686.34},  {495.723, 700.451},
          {851.807, 568.586}, {783.146, 580.028}, {711.791, 591.557}, {637.644, 603.443}, {560.476, 615.572}, {479.795, 628.289},
          {835.828, 503.389}, {767.073, 513.562}, {695.769, 524.066}, {621.661, 534.616}, {544.447, 545.479}, {463.746, 556.585},
          {819.766, 438.353}, {750.967, 447.499}, {679.553, 456.666}, {605.431, 466.013}, {528.161, 475.549}, {447.406, 485.297},
      },
      {
          {558.66, 946.555},  {489.78, 913.384},  {417.544, 879.005}, {341.766, 843.354}, {262.567, 806.015}, {179.219, 767.13},
          {598.202, 883.596}, {530.71, 849.557},  {460.232, 814.384}, {386.347, 777.606}, {308.558, 739.422}, {226.968, 699.328},
          {636.596, 822.004}, {570.495, 787.262}, {501.588, 751.035}, {429.059, 713.458}, {353.069, 674.126}, {273.213, 632.958},
          {674.125, 761.545}, {609.209, 725.954}, {541.478, 688.971}, {470.623, 650.404}, {396.294, 610.102}, {318.045, 567.721},
          {710.567, 702.069}, {646.841, 665.641}, {580.406, 627.812}, {511.036, 588.396}, {438.272, 547.073}, {361.65, 503.664},
          {746.399, 643.502}, {683.637, 606.264}, {618.464, 567.548}, {550.386, 527.183}, {479.156, 484.876}, {403.985, 440.412},
          {781.439, 585.547}, {719.861, 547.561}, {655.687, 508},     {588.744, 466.578}, {518.656, 423.363}, {445.112, 377.76},
      },
      {
          {2010, 952},        {1962.73, 959.47},  {1910.61, 965.105}, {1857.98, 970.642}, {1805.33, 976.519}, {1751.98, 982.468},
          {1969.02, 905.662}, {1918.77, 910.634}, {1868.3, 915.775},  {1817.51, 921.069}, {1766.43, 926.458}, {1714.62, 931.804},
          {1926.55, 860.497}, {1878.01, 865.253}, {1829.14, 869.907}, {1779.99, 874.719}, {1730.31, 879.612}, {1680, 884.616},
          {1887.36, 818.351}, {1840.04, 822.543}, {1792.68, 826.957}, {1744.73, 831.384}, {1696.54, 835.79},  {1647.59, 840.41},
          {1850.49, 778.523}, {1804.81, 782.51},  {1758.56, 786.473}, {1712.03, 790.549}, {1665.15, 794.649}, {1617.51, 798.778},
          {1816.35, 741.022}, {1771.75, 744.604}, {1726.76, 748.416}, {1681.38, 752.244}, {1635.5, 755.834},  {1589.28, 759.66},
          {1784.39, 705.578}, {1740.77, 709},     {1696.96, 712.44},  {1652.61, 715.738}, {1607.85, 719.327}, {1562.78, 722.724},
      },
      {
          {1129.61, 736.821}, {1037.34, 726.818}, {944.529, 716.679}, {850.754, 706.405}, {755.823, 695.91},  {659.056, 685.219},
          {1141.07, 643.746}, {1047.81, 633.483}, {954.073, 622.935}, {859.5, 612.042},   {763.619, 600.859}, {665.95, 589.286},
          {1152.92, 548.533}, {1058.56, 538.028}, {963.815, 526.965}, {868.317, 515.506}, {771.403, 503.607}, {672.544, 491.037},
          {1165.25, 450.592}, {1069.67, 439.637}, {973.686, 428.182}, {876.858, 416.047}, {778.72, 403.38},   {678.666, 389.806},
          {1178.06, 349.442}, {1081.28, 338.139}, {983.815, 326.169}, {885.565, 313.292}, {785.98, 299.727},  {684.555, 285.259},
          {1191.48, 244.718}, {1093.05, 232.922}, {994.182, 220.399}, {894.419, 206.672}, {793.231, 192.306}, {690.228, 176.659},
          {1205.73, 135.313}, {1105.53, 123.008}, {1004.86, 109.929}, {903.311, 95.44},   {800.319, 79.9359}, {695.479, 63.368},
      },
      {
          {1101.65, 609.098}, {1052.23, 626.368}, {1003.22, 643.367}, {954.733, 660.144}, {906.823, 676.645}, {859.35, 693.038},
          {1085.96, 560.822}, {1036.86, 578.382}, {988.353, 595.476}, {940.218, 612.41},  {892.574, 629.098}, {845.352, 645.575},
          {1070.5, 513.158},  {1021.64, 530.792}, {973.502, 548.172}, {925.705, 565.211}, {878.451, 582.03},  {831.483, 598.621},
          {1055.27, 465.629}, {1006.74, 483.601}, {958.773, 501.132}, {911.435, 518.389}, {864.42, 535.323},  {817.73, 552.123},
          {1040.04, 418.537}, {991.801, 436.64},  {944.259, 454.4},   {897.148, 471.793}, {850.426, 488.932}, {804.066, 505.855},
          {1025.09, 371.687}, {977.165, 389.991}, {929.704, 407.963}, {882.9, 425.561},   {836.452, 442.849}, {790.371, 459.859},
          {1010.17, 325.216}, {962.508, 343.635}, {915.4, 361.686},   {868.671, 379.543}, {822.527, 397.044}, {776.671, 414.258},
      },
      {
          {1056.94, 571.152}, {1024.3, 573.757},  {991.813, 576.346}, {959.601, 578.767}, {927.573, 581.336}, {895.644, 583.679},
          {1055.53, 539.438}, {1023, 542.093},    {990.661, 544.776}, {958.568, 547.412}, {926.598, 549.88},  {894.851, 552.436},
          {1054.21, 507.779}, {1021.75, 510.646}, {989.545, 513.444}, {957.54, 516.067},  {925.685, 518.67},  {894.088, 521.297},
          {1052.81, 476.36},  {1020.51, 479.366}, {988.405, 482.16},  {956.497, 484.934}, {924.677, 487.622}, {893.195, 490.358},
          {1051.5, 445.02},   {1019.3, 448.096},  {987.251, 451.025}, {955.43, 453.81},   {923.728, 456.646}, {892.295, 459.465},
          {1050.19, 413.829}, {1018.02, 416.977}, {986.072, 419.999}, {954.331, 422.896}, {922.73, 425.874},  {891.39, 428.612},
          {1048.88, 382.729}, {1016.77, 385.954}, {984.953, 389.06},  {953.322, 392.169}, {921.766, 395.166}, {890.502, 398.035},
      },
      {
          {1148.13, 917.896}, {1048.53, 922.047}, {950.984, 926.4},   {855.018, 930.972}, {760.258, 935.495}, {666.037, 940.416},
          {1142.47, 818.217}, {1044.42, 823.425}, {948.353, 828.585}, {853.713, 833.7},   {760.338, 839.035}, {667.563, 844.245},
          {1137.17, 721.471}, {1040.4, 727.637},  {945.611, 733.57},  {852.354, 739.471}, {760.169, 745.192}, {668.528, 750.792},
          {1132.12, 626.882}, {1036.52, 634.029}, {942.817, 640.818}, {850.586, 647.361}, {759.516, 653.583}, {669.012, 659.664},
          {1127.38, 534.281}, {1032.72, 542.38},  {940.06, 549.849},  {848.751, 557.037}, {758.588, 563.821}, {668.964, 570.282},
          {1122.8, 442.91},   {1029.05, 451.978}, {937.181, 460.343}, {846.672, 468.063}, {757.427, 475.43},  {668.589, 482.284},
          {1118.49, 352.667}, {1025.47, 362.626}, {934.286, 371.658}, {844.464, 380.225}, {755.787, 388.13},  {667.792, 395.408},
      },
      {
          {942.463, 1397.49}, {844.914, 1338.27}, {743.47, 1277.8},   {636.999, 1216.47}, {524.644, 1152.78}, {405.15, 1086.77},
          {1009.93, 1288.31}, {915.571, 1228.64}, {817.646, 1167.6},  {715.401, 1104.81}, {607.64, 1040.17},  {493.277, 972.508},
          {1074.69, 1184.86}, {983.349, 1124.45}, {888.488, 1062.56}, {789.726, 998.85},  {685.909, 933.014}, {575.833, 863.915},
          {1137.4, 1085.58},  {1048.38, 1024.52}, {956.457, 962.018}, {860.756, 897.393}, {760.523, 830.225}, {654.478, 759.647},
          {1198.4, 990.004},  {1111.59, 928.432}, {1022.09, 865.001}, {929.252, 799.387}, {832.01, 730.97},   {729.493, 659.008},
          {1258.09, 897.337}, {1173.38, 835.056}, {1086.08, 770.663}, {995.522, 704.044}, {901.184, 634.325}, {801.672, 560.789},
          {1316.93, 806.933}, {1233.98, 743.835}, {1148.66, 678.599}, {1060.41, 610.72},  {968.39, 539.655},  {871.664, 464.552},
      }};
}