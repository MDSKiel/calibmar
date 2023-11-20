#include "colmap/estimators/generalized_pose.h"
#include "colmap/estimators/two_view_geometry.h"
#include "colmap/geometry/pose.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/math/math.h"
#include "colmap/math/random.h"
#include "colmap/scene/camera.h"

#include <fstream>

using namespace colmap;

struct PointsData {
  std::vector<Eigen::Vector2d> points2D1;
  std::vector<Eigen::Vector2d> points2D1_refrac;
  std::vector<Eigen::Vector2d> points2D2;
  std::vector<Eigen::Vector2d> points2D2_refrac;

  // Store virtual cameras here.
  std::vector<Camera> virtual_cameras1;
  std::vector<Camera> virtual_cameras2;
  std::vector<Rigid3d> virtual_from_reals1;
  std::vector<Rigid3d> virtual_from_reals2;

  colmap::Rigid3d cam2_from_cam1_gt;
};

void GenerateRandom2D2DPoints(const Camera& camera,
                              size_t num_points,
                              const Rigid3d& cam2_from_cam1_gt,
                              PointsData& points_data,
                              double noise_level,
                              double inlier_ratio) {
  // Generate refractive image points first, because flatport reduces FOV.

  points_data.points2D1.clear();
  points_data.points2D1_refrac.clear();
  points_data.points2D2.clear();
  points_data.points2D2_refrac.clear();

  points_data.points2D1.reserve(num_points);
  points_data.points2D1_refrac.reserve(num_points);
  points_data.points2D2.reserve(num_points);
  points_data.points2D2_refrac.reserve(num_points);

  points_data.cam2_from_cam1_gt = cam2_from_cam1_gt;

  size_t num_inliers =
      static_cast<size_t>(static_cast<double>(num_points) * inlier_ratio);
  size_t cnt = 0;
  while (true) {
    if (cnt >= num_points) {
      break;
    }
    Eigen::Vector2d point2D1_refrac;
    point2D1_refrac.x() =
        RandomUniformReal(0.5, static_cast<double>(camera.Width()) - 0.5);
    point2D1_refrac.y() =
        RandomUniformReal(0.5, static_cast<double>(camera.Height()) - 0.5);

    Ray3D ray_refrac = camera.CamFromImgRefrac(point2D1_refrac);

    const double d = RandomUniformReal(0.5, 10.0);

    // Now, do projection.
    Eigen::Vector3d point3D1 = ray_refrac.At(d);
    Eigen::Vector3d point3D2 = cam2_from_cam1_gt * point3D1;

    Eigen::Vector2d point2D2_refrac = camera.ImgFromCamRefrac(point3D2);

    if (std::isnan(point2D2_refrac.x()) || std::isnan(point2D2_refrac.y())) {
      continue;
    }

    if (point2D2_refrac.x() < 0 || point2D2_refrac.x() > camera.Width() ||
        point2D2_refrac.y() < 0 || point2D2_refrac.y() > camera.Height()) {
      continue;
    }

    Eigen::Vector2d point2D1 = camera.ImgFromCam(point3D1.hnormalized());
    Eigen::Vector2d point2D2 = camera.ImgFromCam(point3D2.hnormalized());

    if (cnt < num_inliers) {
      // Add noise to the points.
      if (noise_level > 0) {
        point2D1.x() += RandomGaussian(0.0, noise_level);
        point2D1.y() += RandomGaussian(0.0, noise_level);
        point2D1_refrac.x() += RandomGaussian(0.0, noise_level);
        point2D1_refrac.y() += RandomGaussian(0.0, noise_level);

        point2D2.x() += RandomGaussian(0.0, noise_level);
        point2D2.y() += RandomGaussian(0.0, noise_level);
        point2D2_refrac.x() += RandomGaussian(0.0, noise_level);
        point2D2_refrac.y() += RandomGaussian(0.0, noise_level);
      }
    } else {
      // Add huge noise to the points, this should be an outlier point.
      point2D1.x() += RandomGaussian(0.0, 200.0);
      point2D1.y() += RandomGaussian(0.0, 200.0);
      point2D1_refrac.x() += RandomGaussian(0.0, 200.0);
      point2D1_refrac.y() += RandomGaussian(0.0, 200.0);

      point2D2.x() += RandomGaussian(0.0, 200.0);
      point2D2.y() += RandomGaussian(0.0, 200.0);
      point2D2_refrac.x() += RandomGaussian(0.0, 200.0);
      point2D2_refrac.y() += RandomGaussian(0.0, 200.0);
    }

    points_data.points2D1.push_back(point2D1);
    points_data.points2D2.push_back(point2D2);
    points_data.points2D1_refrac.push_back(point2D1_refrac);
    points_data.points2D2_refrac.push_back(point2D2_refrac);

    cnt++;
  }

  camera.ComputeVirtuals(points_data.points2D1_refrac,
                         points_data.virtual_cameras1,
                         points_data.virtual_from_reals1);
  camera.ComputeVirtuals(points_data.points2D2_refrac,
                         points_data.virtual_cameras2,
                         points_data.virtual_from_reals2);
}

size_t EstimateRelativePose(Camera& camera,
                            const PointsData& points_data,
                            Rigid3d& cam2_from_cam1,
                            bool is_refractive) {
  size_t num_points = points_data.points2D1.size();
  size_t num_inliers = 0;

  TwoViewGeometryOptions two_view_geometry_options;
  two_view_geometry_options.compute_relative_pose = true;
  two_view_geometry_options.ransac_options.max_error = 4.0;

  FeatureMatches matches;
  matches.reserve(num_points);

  for (size_t i = 0; i < num_points; i++) {
    matches.emplace_back(i, i);
  }

  TwoViewGeometry two_view_geometry;

  if (!is_refractive) {
    two_view_geometry =
        EstimateCalibratedTwoViewGeometry(camera,
                                          points_data.points2D1,
                                          camera,
                                          points_data.points2D2,
                                          matches,
                                          two_view_geometry_options);
    cam2_from_cam1 = two_view_geometry.cam2_from_cam1;
    num_inliers = two_view_geometry.inlier_matches.size();
  } else {
    // Refractive case.
    two_view_geometry_options.compute_relative_pose = false;
    two_view_geometry =
        EstimateRefractiveTwoViewGeometry(points_data.points2D1_refrac,
                                          points_data.virtual_cameras1,
                                          points_data.virtual_from_reals1,
                                          points_data.points2D2_refrac,
                                          points_data.virtual_cameras2,
                                          points_data.virtual_from_reals2,
                                          matches,
                                          two_view_geometry_options);
  }

  cam2_from_cam1 = two_view_geometry.cam2_from_cam1;
  num_inliers = two_view_geometry.inlier_matches.size();

  return num_inliers;
}

void RelativePoseError(const colmap::Rigid3d& cam2_from_cam1_gt,
                       const colmap::Rigid3d& cam2_from_cam1_est,
                       double& rotation_error,
                       double& angular_error,
                       double& scale_error,
                       bool is_refractive) {
  colmap::Rigid3d cam2_from_cam1_gt_norm = cam2_from_cam1_gt;
  cam2_from_cam1_gt_norm.translation.normalize();
  colmap::Rigid3d cam2_from_cam1_est_norm = cam2_from_cam1_est;
  cam2_from_cam1_est_norm.translation.normalize();

  colmap::Rigid3d diff =
      cam2_from_cam1_gt_norm * colmap::Inverse(cam2_from_cam1_est_norm);
  rotation_error = colmap::RadToDeg(Eigen::AngleAxisd(diff.rotation).angle());

  double cos_theta = cam2_from_cam1_gt_norm.translation.dot(
      cam2_from_cam1_est_norm.translation);
  if (cos_theta < 0) {
    cos_theta = -cos_theta;
  }
  angular_error = RadToDeg(acos(cos_theta));

  if (is_refractive) {
    const double baseline_est = (cam2_from_cam1_est.rotation.inverse() *
                                 -cam2_from_cam1_est.translation)
                                    .norm();
    const double baseline_gt =
        (cam2_from_cam1_gt.rotation.inverse() * -cam2_from_cam1_gt.translation)
            .norm();
    scale_error = (std::abs(baseline_gt - baseline_est) / baseline_gt) * 100.0;
  } else {
    scale_error = 0.0;
  }
}

void Evaluate(colmap::Camera& camera,
              size_t num_points,
              size_t num_exps,
              double inlier_ratio,
              const std::string& output_path) {
  std::vector<double> noise_levels = {0.0, 0.2, 0.5, 0.8, 1.2, 1.5, 1.8, 2.0};
  // std::vector<double> noise_levels = {0.0};

  std::ofstream file(output_path, std::ios::out);
  file << "# noise_level angular_error_mean angular_error_std rot_error_mean "
          "rot_error_std angular_error_refrac_mean angular_error_refrac_std "
          "rot_error_refrac_mean rot_error_refrac_std time time_refrac"
       << std::endl;

  for (const double& noise : noise_levels) {
    std::cout << "Noise level: " << noise << std::endl;

    // Generate random datasets first.
    std::vector<PointsData> datasets;
    datasets.reserve(num_exps);

    std::cout << "Generating random data ..." << std::endl;
    for (size_t i = 0; i < num_exps; i++) {
      // Create a random GT pose.
      double ry = colmap::RandomUniformReal(colmap::DegToRad(-15.0),
                                            colmap::DegToRad(15.0));
      double rx = colmap::RandomUniformReal(colmap::DegToRad(-15.0),
                                            colmap::DegToRad(15.0));
      double rz = colmap::RandomUniformReal(colmap::DegToRad(-15.0),
                                            colmap::DegToRad(15.0));
      double tx = colmap::RandomUniformReal(-1.0, 1.0);
      double ty = colmap::RandomUniformReal(-0.2, 0.2);
      double tz = colmap::RandomUniformReal(-0.2, 0.2);
      colmap::Rigid3d cam2_from_cam1;
      cam2_from_cam1.rotation =
          Eigen::Quaterniond(EulerAnglesToRotationMatrix(rx, ry, rz))
              .normalized();
      cam2_from_cam1.translation = Eigen::Vector3d(tx, ty, tz);

      PointsData points_data;
      GenerateRandom2D2DPoints(
          camera, num_points, cam2_from_cam1, points_data, noise, inlier_ratio);
      datasets.push_back(points_data);
    }

    std::cout << "Evaluating ..." << std::endl;

    // Evaluate random dataset.
    std::vector<double> rotation_errors;
    std::vector<double> angular_errors;
    std::vector<double> rotation_errors_refrac;
    std::vector<double> angular_errors_refrac;

    std::vector<double> scale_errors;

    // Inlier ratio
    std::vector<double> inlier_ratios;
    std::vector<double> inlier_ratios_refrac;

    double time = 0.0;
    double time_refrac = 0.0;

    Timer timer;

    // Perform in-air pose estimation
    timer.Start();
    for (size_t i = 0; i < num_exps; i++) {
      const PointsData& points_data = datasets[i];
      colmap::Rigid3d cam2_from_cam1_est;
      size_t num_inliers =
          EstimateRelativePose(camera, points_data, cam2_from_cam1_est, false);

      double rotation_error, angular_error, scale_error;
      RelativePoseError(points_data.cam2_from_cam1_gt,
                        cam2_from_cam1_est,
                        rotation_error,
                        angular_error,
                        scale_error,
                        false);
      rotation_errors.push_back(rotation_error);
      angular_errors.push_back(angular_error);
      inlier_ratios.push_back(static_cast<double>(num_inliers) /
                              static_cast<double>(num_points));
    }
    timer.Pause();
    time = timer.ElapsedSeconds();

    timer.Restart();
    // Perform refractive pose estimation
    for (size_t i = 0; i < num_exps; i++) {
      const PointsData& points_data = datasets[i];
      colmap::Rigid3d cam2_from_cam1_est_refrac;
      size_t num_inliers = EstimateRelativePose(
          camera, points_data, cam2_from_cam1_est_refrac, true);

      double rotation_error_refrac, angular_error_refrac, scale_error;
      RelativePoseError(points_data.cam2_from_cam1_gt,
                        cam2_from_cam1_est_refrac,
                        rotation_error_refrac,
                        angular_error_refrac,
                        scale_error,
                        true);
      rotation_errors_refrac.push_back(rotation_error_refrac);
      angular_errors_refrac.push_back(angular_error_refrac);
      scale_errors.push_back(scale_error);
      inlier_ratios_refrac.push_back(static_cast<double>(num_inliers) /
                                     static_cast<double>(num_points));
    }
    timer.Pause();
    time_refrac = timer.ElapsedSeconds();

    const double ang_error_mean = Mean(angular_errors);
    const double ang_error_std = StdDev(angular_errors);
    const double rot_error_mean = Mean(rotation_errors);
    const double rot_error_std = StdDev(rotation_errors);

    const double ang_error_refrac_mean = Mean(angular_errors_refrac);
    const double ang_error_refrac_std = StdDev(angular_errors_refrac);
    const double rot_error_refrac_mean = Mean(rotation_errors_refrac);
    const double rot_error_refrac_std = StdDev(rotation_errors_refrac);

    const double scale_error_mean = Mean(scale_errors);
    const double scale_error_std = StdDev(scale_errors);

    const double inlier_ratio_mean = colmap::Mean(inlier_ratios);
    const double inlier_ratio_refrac_mean = colmap::Mean(inlier_ratios_refrac);

    file << noise << " " << ang_error_mean << " " << ang_error_std << " "
         << rot_error_mean << " " << rot_error_std << " "
         << ang_error_refrac_mean << " " << ang_error_refrac_std << " "
         << rot_error_refrac_mean << " " << rot_error_refrac_std << " "
         << scale_error_mean << " " << scale_error_std << " " << time << " "
         << time_refrac << " " << inlier_ratio_mean << " "
         << inlier_ratio_refrac_mean << std::endl;
    std::cout << "Pose error in-air: Rotation: " << rot_error_mean << " +/- "
              << rot_error_std << " -- Angular: " << ang_error_mean << " +/- "
              << ang_error_std << " -- inlier ratio: " << inlier_ratio_mean
              << " GT inlier ratio: " << inlier_ratio << std::endl;
    std::cout << "Pose error refrac: Rotation: " << rot_error_refrac_mean
              << " +/- " << rot_error_refrac_std
              << " -- Angular: " << ang_error_refrac_mean << " +/- "
              << ang_error_refrac_std << " -- Scale: " << scale_error_mean
              << " +/- " << scale_error_std
              << " -- inlier ratio: " << inlier_ratio_refrac_mean
              << " GT inlier ratio: " << inlier_ratio << std::endl;
  }

  file.close();
}

int main(int argc, char* argv[]) {
  SetPRNGSeed(time(NULL));

  std::cout << "Setup some realistic camera model" << std::endl;

  // Camera parameters coming from Anton 131 map.
  Camera camera;
  camera.SetWidth(4104);
  camera.SetHeight(3006);
  camera.SetModelIdFromName("METASHAPE_FISHEYE");
  std::vector<double> params = {1000.7964068878537,
                                1000.6679248258547,
                                2097.4832274550317,
                                1641.1207545881762,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0};
  // Camera camera;
  // camera.SetWidth(2048);
  // camera.SetHeight(1536);
  // camera.SetModelIdFromName("PINHOLE");
  // std::vector<double> params = {
  //     1300.900000, 1300.900000, 1024.000000, 768.000000};

  camera.SetParams(params);

  // Flatport setup.
  camera.SetRefracModelIdFromName("FLATPORT");
  Eigen::Vector3d int_normal;
  int_normal[0] = RandomUniformReal(-0.3, 0.3);
  int_normal[1] = RandomUniformReal(-0.3, 0.3);
  int_normal[2] = RandomUniformReal(0.7, 1.3);

  int_normal.normalize();

  std::vector<double> flatport_params = {
      int_normal[0], int_normal[1], int_normal[2], 2.0, 1.0, 1.0, 2.5, 1.8};
  camera.SetRefracParams(flatport_params);

  // Generate simulated point data.
  const size_t num_points = 100;
  const double inlier_ratio = 1.0;

  std::string output_dir =
      "/home/mshe/workspace/omv_src/colmap-project/refrac_sfm_eval/data/"
      "rel_pose/";
  std::stringstream ss;
  ss << output_dir << "/eval_refrac_rel_pose_num_points_" << num_points
     << "_inlier_ratio_" << inlier_ratio << ".txt";
  std::string output_path = ss.str();

  Evaluate(camera, num_points, 20, inlier_ratio, output_path);

  return true;
}