#include "stereo_calibration.h"
#include "general_calibration.h"

#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/homography_matrix.h>

#include <thread>

namespace {
  using namespace calibmar;

  // Estimate poses from a calibrated camera (with known intrinsics). Using homographies from planar calibration target, as
  // done in general calibration.
  void EstimatePosesForCamera(const colmap::Camera& camera, const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                              const std::vector<std::vector<Eigen::Vector2d>>& image_points,
                              std::vector<colmap::Rigid3d>& poses) {
    // create 2D plane points
    std::vector<std::vector<Eigen::Vector2d>> object_plane_points(object_points.size());
    for (size_t i = 0; i < object_points.size(); i++) {
      object_plane_points[i].reserve(object_points[i].size());
      for (size_t j = 0; j < object_points[i].size(); j++) {
        const auto& point_obj = object_points[i][j];
        object_plane_points[i].push_back({point_obj.x(), point_obj.y()});
      }
    }

    const std::vector<std::vector<Eigen::Vector2d>>* undistorted_points_ptr;
    std::vector<std::vector<Eigen::Vector2d>> undistorted_points;
    undistorted_points_ptr = &undistorted_points;

    if (!camera.IsUndistorted()) {
      general_calibration::UndistortImagePoints(image_points, camera, undistorted_points);
    }
    else {
      undistorted_points_ptr = &image_points;
    }

    poses.reserve(image_points.size());
    for (size_t i = 0; i < image_points.size(); i++) {
      if (object_points[i].size() != image_points[i].size()) {
        throw std::runtime_error("Image points object points size missmatch!");
      }
      Eigen::Matrix3d H = colmap::HomographyMatrixEstimator::Estimate(object_plane_points[i], (*undistorted_points_ptr)[i])[0];

      colmap::Rigid3d pose;
      general_calibration::EstimatePoseFromHomography(H, camera.CalibrationMatrix(), pose);

      poses.push_back(pose);
    }
  }

  // Estimate a relative pose from two sets of absolute poses which should be constant relative to each other
  // Uses the median of each translation component and rotation vector component to form the relative pose
  // This is seems to be what opencv does aswell. This is robust towards outliers or flipped poses as compared to average
  colmap::Rigid3d EstimateRelativePose(std::vector<colmap::Rigid3d> abs_poses1, std::vector<colmap::Rigid3d> abs_poses2) {
    std::vector<Eigen::AngleAxisd> rotations;
    std::vector<Eigen::Vector3d> translations;

    // x, y, z, x_rot, y_rot, z_rot, angle
    std::vector<std::vector<double>> components(7);
    for (std::vector<double>& vec : components) {
      vec.resize(abs_poses1.size());
    }

    size_t n = abs_poses1.size();
    for (size_t i = 0; i < n; i++) {
      colmap::Rigid3d& pose1 = abs_poses1[i];
      colmap::Rigid3d& pose2 = abs_poses2[i];
      colmap::Rigid3d relative = pose2 * colmap::Inverse(pose1);
      Eigen::AngleAxisd angle_ax(relative.rotation);

      components[0][i] = relative.translation.x();
      components[1][i] = relative.translation.y();
      components[2][i] = relative.translation.z();
      components[3][i] = angle_ax.axis().x();
      components[4][i] = angle_ax.axis().y();
      components[5][i] = angle_ax.axis().z();
      components[6][i] = angle_ax.angle();
    }

    size_t median_idx = n / 2;
    for (std::vector<double>& vec : components) {
      // sort up to median_idx
      std::nth_element(vec.begin(), vec.begin() + median_idx, vec.end());
    }

    bool even = n % 2 == 0;
    colmap::Rigid3d relative;
    relative.translation.x() = even ? components[0][median_idx] : (components[0][median_idx] + components[0][median_idx + 1]) / 2;
    relative.translation.y() = even ? components[1][median_idx] : (components[1][median_idx] + components[1][median_idx + 1]) / 2;
    relative.translation.z() = even ? components[2][median_idx] : (components[2][median_idx] + components[2][median_idx + 1]) / 2;
    Eigen::Vector3d rot_vec;
    rot_vec.x() = even ? components[3][median_idx] : (components[3][median_idx] + components[3][median_idx + 1]) / 2;
    rot_vec.y() = even ? components[4][median_idx] : (components[4][median_idx] + components[4][median_idx + 1]) / 2;
    rot_vec.z() = even ? components[5][median_idx] : (components[5][median_idx] + components[5][median_idx + 1]) / 2;
    double angle = even ? components[6][median_idx] : (components[6][median_idx] + components[6][median_idx + 1]) / 2;

    relative.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot_vec.normalized()));
    return relative;
  }

  void AddImageToProblem(ceres::Problem& problem, colmap::Camera& camera, colmap::Rigid3d& absolute_pose,
                         colmap::Rigid3d& relative_pose, const std::vector<Eigen::Vector2d>& image_points,
                         std::vector<Eigen::Vector3d>& object_points) {
    double* absolute_rotation = absolute_pose.rotation.coeffs().data();
    double* absolute_translation = absolute_pose.translation.data();
    double* relative_rotation = relative_pose.rotation.coeffs().data();
    double* relative_translation = relative_pose.translation.data();
    double* camera_params = camera.ParamsData();

    ceres::CostFunction* cost_function;

    for (size_t i = 0; i < image_points.size(); i++) {
      const auto& point2D = image_points[i];
      auto& point3D = object_points[i];

      switch (camera.ModelId()) {
#define CAMERA_MODEL_CASE(CameraModel)                                                        \
  case colmap::CameraModel::kModelId:                                                         \
    cost_function = colmap::RigReprojErrorCostFunction<colmap::CameraModel>::Create(point2D); \
    break;

        CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
      }

      problem.AddResidualBlock(cost_function, nullptr, relative_rotation, relative_translation, absolute_rotation,
                               absolute_translation, point3D.data(), camera_params);

      problem.SetParameterBlockConstant(point3D.data());
    }

    colmap::SetQuaternionManifold(&problem, relative_rotation);
    colmap::SetQuaternionManifold(&problem, absolute_rotation);
  }

  void GetEstimatedStdDeviation(ceres::Covariance& covariance, double* params, size_t num_params,
                                std::vector<double>& std_deviations) {
    std_deviations.clear();
    std_deviations.reserve(num_params);
    std::vector<double> covariance_mat(num_params * num_params);

    if (covariance.GetCovarianceBlock(params, params, covariance_mat.data())) {
      for (size_t i = 0; i < num_params; i++) {
        // diagonal indices
        size_t idx = i + i * num_params;
        double std = covariance_mat[idx] == 0 ? 0 : sqrt(covariance_mat[idx]);
        std_deviations.push_back(std);
      }
    }
  }
}

namespace calibmar::stereo_calibration {
  void CalibrateStereoCameras(std::vector<std::vector<Eigen::Vector3d>>& object_points,
                              const std::vector<std::vector<Eigen::Vector2d>>& image_points1,
                              const std::vector<std::vector<Eigen::Vector2d>>& image_points2, colmap::Camera& camera1,
                              colmap::Camera& camera2, bool use_intrinsic_guess, bool fix_intrinsics,
                              colmap::Rigid3d& relative_pose, std::vector<colmap::Rigid3d>& poses,
                              StereoStdDeviations* std_deviations) {
    if (object_points.size() != image_points1.size() || image_points1.size() != image_points2.size()) {
      throw std::runtime_error("Object and image point sizes must match!");
    }

    std::vector<colmap::Rigid3d> poses2;
    if (!fix_intrinsics) {
      general_calibration::CalibrateCamera(object_points, image_points1, camera1, use_intrinsic_guess, poses);
      general_calibration::CalibrateCamera(object_points, image_points2, camera2, use_intrinsic_guess, poses2);
    }
    else {
      EstimatePosesForCamera(camera1, object_points, image_points1, poses);
      EstimatePosesForCamera(camera2, object_points, image_points2, poses2);
    }

    relative_pose = EstimateRelativePose(poses, poses2);

    ceres::Problem problem;
    colmap::Rigid3d identity;

    for (size_t i = 0; i < object_points.size(); i++) {
      AddImageToProblem(problem, camera1, poses[i], identity, image_points1[i], object_points[i]);
    }
    for (size_t i = 0; i < object_points.size(); i++) {
      // camera1 poses are absolute poses
      AddImageToProblem(problem, camera2, poses[i], relative_pose, image_points2[i], object_points[i]);
    }

    // Keep first "relative" pose constant since camera1 is origin of rig
    problem.SetParameterBlockConstant(identity.rotation.coeffs().data());
    problem.SetParameterBlockConstant(identity.translation.data());

    if (fix_intrinsics) {
      problem.SetParameterBlockConstant(camera1.ParamsData());
      problem.SetParameterBlockConstant(camera2.ParamsData());
    }

    // in case we have simple models keep the principal point constant
    if (camera1.ModelId() == colmap::PinholeCameraModel::model_id ||
        camera1.ModelId() == colmap::SimplePinholeCameraModel::model_id) {
      std::vector<int> const_camera_params;
      const std::vector<size_t>& params_idxs = camera1.PrincipalPointIdxs();
      const_camera_params.insert(const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      colmap::SetSubsetManifold(static_cast<int>(camera1.NumParams()), const_camera_params, &problem, camera1.ParamsData());
    }
    if (camera2.ModelId() == colmap::PinholeCameraModel::model_id ||
        camera2.ModelId() == colmap::SimplePinholeCameraModel::model_id) {
      std::vector<int> const_camera_params;
      const std::vector<size_t>& params_idxs = camera2.PrincipalPointIdxs();
      const_camera_params.insert(const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      colmap::SetSubsetManifold(static_cast<int>(camera2.NumParams()), const_camera_params, &problem, camera2.ParamsData());
    }

    // Solve
    ceres::Solver::Options solver_options;
    solver_options.gradient_tolerance = 1e-20;
    solver_options.function_tolerance = 1e-20;
    solver_options.max_num_iterations = 100;
    solver_options.linear_solver_type = ceres::DENSE_SCHUR;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.num_threads = std::thread::hardware_concurrency();

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    if (std_deviations) {
      ceres::Covariance::Options covariance_options;
      covariance_options.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
      covariance_options.null_space_rank = -1;
      ceres::Covariance covariance(covariance_options);

      // check which parameters are requested
      std::vector<std::pair<const double*, const double*>> requested_params;
      if (std_deviations->std_deviations_intrinsics1) {
        requested_params.push_back({camera1.ParamsData(), camera1.ParamsData()});
      }
      if (std_deviations->std_deviations_intrinsics2) {
        requested_params.push_back({camera2.ParamsData(), camera2.ParamsData()});
      }
      if (std_deviations->std_deviations_extrinsics1) {
        // this should always be one, but calculate it anyway
        requested_params.push_back({identity.rotation.coeffs().data(), identity.rotation.coeffs().data()});
        requested_params.push_back({identity.translation.data(), identity.translation.data()});
      }
      if (std_deviations->std_deviations_extrinsics2) {
        requested_params.push_back({relative_pose.rotation.coeffs().data(), relative_pose.rotation.coeffs().data()});
        requested_params.push_back({relative_pose.translation.data(), relative_pose.translation.data()});
      }

      // compute the requested covariance matrix parts and then extract the std deviation from it
      if (covariance.Compute(requested_params, &problem)) {
        if (std_deviations->std_deviations_intrinsics1) {
          GetEstimatedStdDeviation(covariance, camera1.ParamsData(), camera1.NumParams(),
                                   *std_deviations->std_deviations_intrinsics1);
        }
        if (std_deviations->std_deviations_intrinsics2) {
          GetEstimatedStdDeviation(covariance, camera2.ParamsData(), camera2.NumParams(),
                                   *std_deviations->std_deviations_intrinsics2);
        }
        if (std_deviations->std_deviations_extrinsics1) {
          std::vector<double> rotation_std;
          std::vector<double> translation_std;

          GetEstimatedStdDeviation(covariance, identity.rotation.coeffs().data(), identity.rotation.coeffs().size(),
                                   rotation_std);
          GetEstimatedStdDeviation(covariance, identity.translation.data(), identity.translation.size(), translation_std);

          for (double std : rotation_std) {
            std_deviations->std_deviations_extrinsics1->push_back(std);
          }
          for (double std : translation_std) {
            std_deviations->std_deviations_extrinsics1->push_back(std);
          }
        }
        if (std_deviations->std_deviations_extrinsics2) {
          std::vector<double> rotation_std;
          std::vector<double> translation_std;

          GetEstimatedStdDeviation(covariance, relative_pose.rotation.coeffs().data(), relative_pose.rotation.coeffs().size(),
                                   rotation_std);
          GetEstimatedStdDeviation(covariance, relative_pose.translation.data(), relative_pose.translation.size(),
                                   translation_std);

          for (double std : rotation_std) {
            std_deviations->std_deviations_extrinsics2->push_back(std);
          }
          for (double std : translation_std) {
            std_deviations->std_deviations_extrinsics2->push_back(std);
          }
        }
      }
    }
  }
}
