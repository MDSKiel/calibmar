#include "colmap/estimators/cost_functions.h"
#include "colmap/estimators/generalized_pose.h"
#include "colmap/estimators/refrac_relative_pose.h"
#include "colmap/estimators/two_view_geometry.h"
#include "colmap/geometry/essential_matrix.h"
#include "colmap/geometry/pose.h"
#include "colmap/geometry/rigid3.h"
#include "colmap/geometry/triangulation.h"
#include "colmap/math/math.h"
#include "colmap/math/random.h"
#include "colmap/scene/camera.h"
#include "colmap/scene/projection.h"

#include <fstream>

#include <Eigen/LU>
#include <Eigen/SVD>
#include <unsupported/Eigen/KroneckerProduct>

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
  Camera best_fit_camera;

  colmap::Rigid3d cam2_from_cam1_gt;
};

enum class RelTwoViewMethod {
  kNonRefrac = -1,
  kXiaoHu = 0,
  kXiaoHuRefine = 1,
  kGR6P = 2,
  kGR6PRefine = 3,
  kBestFit = 4,
  kBestFitRefine = 5,
};

void GenerateRandomSecondViewPose(const Eigen::Vector3d& proj_center,
                                  const double distance,
                                  Rigid3d& cam2_from_cam1) {
  Eigen::Vector3d target(0.0, 0.0, distance);
  Eigen::Vector3d zaxis = (target - proj_center).normalized();
  Eigen::Vector3d yaxis(0, -1, 0);
  Eigen::Vector3d xaxis = (zaxis.cross(yaxis)).normalized();
  yaxis = (zaxis.cross(xaxis)).normalized();

  Eigen::Matrix3d R;
  R.col(0) = xaxis;
  R.col(1) = yaxis;
  R.col(2) = zaxis;
  Rigid3d cam1_from_cam2(Eigen::Quaterniond(R), proj_center);
  cam2_from_cam1 = Inverse(cam1_from_cam2);
}

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
        RandomUniformReal(0.5, static_cast<double>(camera.width) - 0.5);
    point2D1_refrac.y() =
        RandomUniformReal(0.5, static_cast<double>(camera.height) - 0.5);

    Ray3D ray_refrac = camera.CamFromImgRefrac(point2D1_refrac);

    const double d = RandomUniformReal(6.0, 8.0);

    // Now, do projection.
    Eigen::Vector3d point3D1 = ray_refrac.At(d);
    Eigen::Vector3d point3D2 = cam2_from_cam1_gt * point3D1;

    Eigen::Vector2d point2D2_refrac = camera.ImgFromCamRefrac(point3D2);

    if (std::isnan(point2D2_refrac.x()) || std::isnan(point2D2_refrac.y())) {
      continue;
    }

    if (point2D2_refrac.x() < 0 || point2D2_refrac.x() > camera.width ||
        point2D2_refrac.y() < 0 || point2D2_refrac.y() > camera.height) {
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

  const double kApproxDepth = 5.0;
  points_data.best_fit_camera =
      BestFitNonRefracCamera(CameraModelId::kOpenCV, camera, kApproxDepth);
}

size_t EstimateRelativePose(Camera& camera,
                            const PointsData& points_data,
                            Rigid3d& cam2_from_cam1,
                            RelTwoViewMethod method_id) {
  size_t num_points = points_data.points2D1.size();

  TwoViewGeometryOptions two_view_geometry_options;
  two_view_geometry_options.compute_relative_pose = true;
  two_view_geometry_options.ransac_options.max_error = 4.0;

  FeatureMatches matches;
  matches.reserve(num_points);

  for (size_t i = 0; i < num_points; i++) {
    matches.emplace_back(i, i);
  }

  TwoViewGeometry two_view_geometry;

  switch (method_id) {
    case RelTwoViewMethod::kNonRefrac:
      two_view_geometry =
          EstimateCalibratedTwoViewGeometry(camera,
                                            points_data.points2D1,
                                            camera,
                                            points_data.points2D2,
                                            matches,
                                            two_view_geometry_options);
      break;
    case RelTwoViewMethod::kGR6P:
      two_view_geometry_options.compute_relative_pose = true;
      two_view_geometry =
          EstimateRefractiveTwoViewGeometry(points_data.points2D1_refrac,
                                            points_data.virtual_cameras1,
                                            points_data.virtual_from_reals1,
                                            points_data.points2D2_refrac,
                                            points_data.virtual_cameras2,
                                            points_data.virtual_from_reals2,
                                            matches,
                                            two_view_geometry_options,
                                            false);
      break;
    case RelTwoViewMethod::kGR6PRefine:
      two_view_geometry_options.compute_relative_pose = true;
      two_view_geometry =
          EstimateRefractiveTwoViewGeometry(points_data.points2D1_refrac,
                                            points_data.virtual_cameras1,
                                            points_data.virtual_from_reals1,
                                            points_data.points2D2_refrac,
                                            points_data.virtual_cameras2,
                                            points_data.virtual_from_reals2,
                                            matches,
                                            two_view_geometry_options,
                                            true);
      break;
    case RelTwoViewMethod::kBestFit:

      two_view_geometry = EstimateRefractiveTwoViewGeometryUseBestFit(
          points_data.best_fit_camera,
          points_data.points2D1_refrac,
          points_data.virtual_cameras1,
          points_data.virtual_from_reals1,
          points_data.best_fit_camera,
          points_data.points2D2_refrac,
          points_data.virtual_cameras2,
          points_data.virtual_from_reals2,
          matches,
          two_view_geometry_options,
          false);
      break;
    case RelTwoViewMethod::kBestFitRefine:
      two_view_geometry = EstimateRefractiveTwoViewGeometryUseBestFit(
          points_data.best_fit_camera,
          points_data.points2D1_refrac,
          points_data.virtual_cameras1,
          points_data.virtual_from_reals1,
          points_data.best_fit_camera,
          points_data.points2D2_refrac,
          points_data.virtual_cameras2,
          points_data.virtual_from_reals2,
          matches,
          two_view_geometry_options,
          true);
      break;
    case RelTwoViewMethod::kXiaoHu:
      two_view_geometry =
          EstimateRefractiveTwoViewGeometryHu(points_data.points2D1_refrac,
                                              points_data.virtual_cameras1,
                                              points_data.virtual_from_reals1,
                                              points_data.points2D2_refrac,
                                              points_data.virtual_cameras2,
                                              points_data.virtual_from_reals2,
                                              matches,
                                              two_view_geometry_options,
                                              false);
      break;
    case RelTwoViewMethod::kXiaoHuRefine:
      two_view_geometry =
          EstimateRefractiveTwoViewGeometryHu(points_data.points2D1_refrac,
                                              points_data.virtual_cameras1,
                                              points_data.virtual_from_reals1,
                                              points_data.points2D2_refrac,
                                              points_data.virtual_cameras2,
                                              points_data.virtual_from_reals2,
                                              matches,
                                              two_view_geometry_options,
                                              true);
      break;
    default:
      LOG(ERROR) << "Relative two-view method does not exist!";
      break;
  }

  cam2_from_cam1 = two_view_geometry.cam2_from_cam1;
  if (two_view_geometry.inlier_matches.size() == 0 ||
      two_view_geometry.tri_angle == 0.0) {
    return 0;
  } else {
    return two_view_geometry.inlier_matches.size();
  }
}

void RelativePoseError(const colmap::Rigid3d& cam2_from_cam1_gt,
                       const colmap::Rigid3d& cam2_from_cam1_est,
                       double& rotation_error,
                       double& angular_error) {
  colmap::Rigid3d cam2_from_cam1_gt_norm = cam2_from_cam1_gt;
  cam2_from_cam1_gt_norm.translation.normalize();
  colmap::Rigid3d cam2_from_cam1_est_norm = cam2_from_cam1_est;
  cam2_from_cam1_est_norm.translation.normalize();

  Eigen::Quaterniond rotation_diff =
      cam2_from_cam1_gt.rotation * cam2_from_cam1_est.rotation.inverse();
  rotation_error = colmap::RadToDeg(Eigen::AngleAxisd(rotation_diff).angle());

  double cos_theta = cam2_from_cam1_gt_norm.translation.dot(
      cam2_from_cam1_est_norm.translation);
  if (cos_theta < 0) {
    cos_theta = -cos_theta;
  }
  if (cos_theta > 1) {
    cos_theta = 1.0;
  }
  angular_error = RadToDeg(acos(cos_theta));
}

void Evaluate(colmap::Camera& camera,
              size_t num_points,
              size_t num_exps,
              double inlier_ratio,
              bool is_flatport,
              const std::vector<RelTwoViewMethod>& methods,
              const std::string& output_path) {
  std::vector<double> noise_levels = {0.0, 0.2, 0.5, 0.8, 1.2, 1.5, 1.8, 2.0};

  // These noise levels are used to evaluate XiaoHu and GR6P approaches.
  // std::vector<double> noise_levels = {0, 0.002, 0.005, 0.008, 0.01};

  std::ofstream file(output_path, std::ios::out);

  for (const double& noise : noise_levels) {
    LOG(INFO) << "Noise level: " << noise;

    // Generate random datasets first.
    std::vector<PointsData> datasets;
    datasets.reserve(num_exps);

    LOG(INFO) << "Generating random data ...";

    for (size_t i = 0; i < num_exps; i++) {
      if (is_flatport) {
        // Flatport setup
        camera.refrac_model_id = CameraRefracModelId::kFlatPort;
        Eigen::Vector3d int_normal;
        int_normal[0] = RandomUniformReal(-0.2, 0.2);
        int_normal[1] = RandomUniformReal(-0.2, 0.2);
        int_normal[2] = RandomUniformReal(0.8, 1.2);
        int_normal.normalize();

        std::vector<double> flatport_params = {
            int_normal[0],
            int_normal[1],
            int_normal[2],
            colmap::RandomUniformReal(0.001, 0.05),
            colmap::RandomUniformReal(0.002, 0.2),
            1.0,
            1.52,
            1.334};
        camera.refrac_params = flatport_params;
      } else {
        camera.refrac_model_id = CameraRefracModelId::kDomePort;
        Eigen::Vector3d decentering;
        decentering[0] = RandomUniformReal(-0.01, 0.01);
        decentering[1] = RandomUniformReal(-0.01, 0.01);
        decentering[2] = RandomUniformReal(-0.03, 0.03);

        std::vector<double> domeport_params = {
            decentering[0],
            decentering[1],
            decentering[2],
            colmap::RandomUniformReal(0.05, 0.07),
            colmap::RandomUniformReal(0.005, 0.02),
            1.0,
            1.52,
            1.334};
        camera.refrac_params = domeport_params;
      }
      // Create a random GT pose.
      const double tx = colmap::RandomUniformReal(8.0, 10.0);
      const double ty = colmap::RandomUniformReal(-2.5, 2.5);
      const double tz = colmap::RandomUniformReal(-2.5, 2.5);
      const double distance = colmap::RandomUniformReal(6.0, 8.0);
      Eigen::Vector3d proj_center(tx, ty, tz);

      colmap::Rigid3d cam2_from_cam1;

      GenerateRandomSecondViewPose(proj_center, distance, cam2_from_cam1);
      PointsData points_data;
      GenerateRandom2D2DPoints(
          camera, num_points, cam2_from_cam1, points_data, noise, inlier_ratio);
      datasets.push_back(points_data);
    }

    // Evaluate random dataset.
    // Errors of all methods:
    std::vector<double> rot_error_mean_all_methods;
    std::vector<double> rot_error_std_all_methods;
    std::vector<double> ang_error_mean_all_methods;
    std::vector<double> ang_error_std_all_methods;
    std::vector<double> inlier_ratio_all_methods;

    for (const RelTwoViewMethod& method_id : methods) {
      // Rotation error (degrees).
      std::vector<double> rotation_errors;
      // Angular error (error between the estimated translation direction and
      // ground truth direction)/
      std::vector<double> angular_errors;
      // Inlier ratio.
      std::vector<double> inlier_ratios;

      LOG(INFO) << "Evaluating Method: " << static_cast<int>(method_id);

      for (size_t i = 0; i < num_exps; i++) {
        const PointsData& points_data = datasets[i];
        colmap::Rigid3d cam2_from_cam1_est;
        size_t num_inliers = EstimateRelativePose(
            camera, points_data, cam2_from_cam1_est, method_id);
        if (num_inliers != 0) {
          double rotation_error, angular_error;
          RelativePoseError(points_data.cam2_from_cam1_gt,
                            cam2_from_cam1_est,
                            rotation_error,
                            angular_error);
          rotation_errors.push_back(rotation_error);
          angular_errors.push_back(angular_error);
          inlier_ratios.push_back(static_cast<double>(num_inliers) /
                                  static_cast<double>(num_points));
        }
      }
      const double rot_error_mean = Mean(rotation_errors);
      const double rot_error_std = StdDev(rotation_errors);
      const double ang_error_mean = Mean(angular_errors);
      const double ang_error_std = StdDev(angular_errors);
      const double inlier_ratio_mean = Mean(inlier_ratios);

      rot_error_mean_all_methods.push_back(rot_error_mean);
      rot_error_std_all_methods.push_back(rot_error_std);
      ang_error_mean_all_methods.push_back(ang_error_mean);
      ang_error_std_all_methods.push_back(ang_error_std);
      inlier_ratio_all_methods.push_back(inlier_ratio_mean);

      LOG(INFO) << "Relative pose error : Rotation: " << rot_error_mean
                << " +/- " << rot_error_std << " -- Angular: " << ang_error_mean
                << " +/- " << ang_error_std
                << " -- inlier ratio: " << inlier_ratio_mean
                << " GT inlier ratio: " << inlier_ratio;
    }

    file << noise;
    for (size_t i = 0; i < methods.size(); i++) {
      file << " " << rot_error_mean_all_methods[i] << " "
           << rot_error_std_all_methods[i] << " "
           << ang_error_mean_all_methods[i] << " "
           << ang_error_std_all_methods[i] << " "
           << inlier_ratio_all_methods[i];
    }
    file << std::endl;
  }

  file.close();
}

int main(int argc, char* argv[]) {
  SetPRNGSeed(time(NULL));

  Camera camera;
  camera.width = 1920;
  camera.height = 1080;
  camera.model_id = CameraModelId::kPinhole;
  std::vector<double> params = {
      1297.3655404279762, 1297.3655404279762, 960.0, 540.0};
  camera.params = params;

  // Generate simulated point data.
  const size_t num_points = 200;
  const double inlier_ratio = 0.7;
  bool is_flatport = false;

  std::string output_dir =
      "/home/mshe/workspace/omv_src/colmap-project/refrac_sfm_eval/plots/"
      "rel_pose/compare_methods3/";
  std::stringstream ss;
  ss << output_dir << "/dome_noise_range_2_" << num_points << "_inlier_ratio_"
     << inlier_ratio << ".txt";
  std::string output_path = ss.str();

  // Which methods to evaluate?
  std::vector<RelTwoViewMethod> methods = {RelTwoViewMethod::kNonRefrac,
                                           RelTwoViewMethod::kXiaoHu,
                                           RelTwoViewMethod::kGR6P,
                                           RelTwoViewMethod::kBestFit};

  Evaluate(
      camera, num_points, 200, inlier_ratio, is_flatport, methods, output_path);

  return true;
}