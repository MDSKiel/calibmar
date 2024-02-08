#include "general_calibration.h"

#include <colmap/estimators/absolute_pose.h>
#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/homography_matrix.h>
#include <colmap/geometry/homography_matrix.h>
#include <colmap/sensor/models.h>

#include <colmap/scene/projection.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace {

  // Compute a single row vector v_ij of V given Homography H
  inline Eigen::Vector<double, 6> v(const Eigen::Matrix3d& H, Eigen::Index i, Eigen::Index j) {
    return {H(0, i) * H(0, j),                      // hi1 hj1
            H(0, i) * H(1, j) + H(1, i) * H(0, j),  // hi1 hj2 + hi2 hj1
            H(1, i) * H(1, j),                      // hi2 hj2
            H(2, i) * H(0, j) + H(0, i) * H(2, j),  // hi3 hj1 + hi1 hj3
            H(2, i) * H(1, j) + H(1, i) * H(2, j),  // hi3 hj2 + hi2 hj3
            H(2, i) * H(2, j)};                     // hi3 hj3
  }

  colmap::Camera CreateUndistortedCamera(colmap::Camera camera) {
    colmap::Camera undistort_camera = colmap::Camera::CreateFromModelId(
        colmap::kInvalidCameraId, colmap::PinholeCameraModel::model_id, 1, camera.width, camera.height);

    undistort_camera.SetPrincipalPointX(camera.PrincipalPointX());
    undistort_camera.SetPrincipalPointY(camera.PrincipalPointY());

    if (camera.FocalLengthIdxs().size() == 1) {
      undistort_camera.SetFocalLength(camera.FocalLength());
    }
    else {
      undistort_camera.SetFocalLengthX(camera.FocalLengthX());
      undistort_camera.SetFocalLengthY(camera.FocalLengthY());
    }

    return undistort_camera;
  }

  void AddImageToProblem(ceres::Problem& problem, colmap::Camera& camera, colmap::Rigid3d& pose,
                         const std::vector<Eigen::Vector2d>& image_points, std::vector<Eigen::Vector3d>& object_points) {
    double* rotation = pose.rotation.coeffs().data();
    double* translation = pose.translation.data();
    double* camera_params = camera.params.data();
    ceres::CostFunction* cost_function;

    for (size_t i = 0; i < image_points.size(); i++) {
      const auto& point2D = image_points[i];
      auto& point3D = object_points[i];

      switch (camera.model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                                     \
  case colmap::CameraModel::model_id:                                                      \
    cost_function = colmap::ReprojErrorCostFunction<colmap::CameraModel>::Create(point2D); \
    break;

        CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
      }

      problem.AddResidualBlock(cost_function, nullptr, rotation, translation, point3D.data(), camera_params);
      problem.SetParameterBlockConstant(point3D.data());
    }

    colmap::SetQuaternionManifold(&problem, rotation);
  }
}

namespace calibmar::general_calibration {
  void UndistortImagePoints(const std::vector<std::vector<Eigen::Vector2d>>& image_points, const colmap::Camera& camera,
                            std::vector<std::vector<Eigen::Vector2d>>& undistorted_points) {
    undistorted_points.clear();
    undistorted_points.resize(image_points.size());

    if (camera.IsUndistorted()) {
      for (size_t i = 0; i < image_points.size(); i++) {
        undistorted_points[i] = image_points[i];
      }
    }
    else {
      colmap::Camera undistort_camera = CreateUndistortedCamera(camera);

      for (size_t i = 0; i < image_points.size(); i++) {
        undistorted_points[i].reserve(image_points[i].size());
        for (const auto& point : image_points[i]) {
          undistorted_points[i].push_back(undistort_camera.ImgFromCam(camera.CamFromImg(point)));
        }
      }
    }
  }

  void EstimateKFromHomographies(const std::vector<std::vector<Eigen::Vector2d>>& object_plane_points,
                                 const std::vector<std::vector<Eigen::Vector2d>>& image_points, double& fx, double& fy,
                                 double& cx, double& cy) {
    if (image_points.size() != object_plane_points.size()) {
      throw std::runtime_error("Different number of views for image and object points!");
    }
    if (image_points.size() < 2) {
      throw std::runtime_error("At least two views needed for calibration");
    }

    Eigen::Matrix<double, Eigen::Dynamic, 6> V;
    V.resize(2 * image_points.size() + 2, Eigen::NoChange);

    for (size_t i = 0; i < image_points.size(); i++) {
      if (image_points[i].size() != object_plane_points[i].size()) {
        throw std::runtime_error("Image points object points size missmatch!");
      }
      if (image_points[i].size() < 4) {
        throw std::runtime_error("Atleast four points needed per view!");
      }

      std::vector<Eigen::Matrix3d> models;
      colmap::HomographyMatrixEstimator::Estimate(object_plane_points[i], image_points[i], &models);
      Eigen::Matrix3d H = models[0];
      H /= H(2, 2);  // potentially unnecessary normalization

      V.row(2 * i) = v(H, 0, 1);
      V.row(2 * i + 1) = (v(H, 0, 0) - v(H, 1, 1));
    }

    V.row(V.rows() - 2) = Eigen::Vector<double, 6>{0, 1, 0, 0, 0, 0};   // add skewless constraint
    V.row(V.rows() - 1) = Eigen::Vector<double, 6>{1, 0, -1, 0, 0, 0};  // add aspect ratio 1 constraint

    Eigen::VectorXd rhs;
    rhs.resize(V.rows());
    rhs.setZero();

    Eigen::JacobiSVD svd(V, Eigen::ComputeFullV);
    Eigen::Vector<double, 6> b = svd.matrixV().col(5);

    // b = [B11 , B12 , B22 , B13 , B23 , B33 ]
    cy = (b(1) * b(3) - b(0) * b(4)) / (b(0) * b(2) - b(1) * b(1));
    double lambda = b(5) - ((b(3) * b(3) + cy * (b(1) * b(3) - b(0) * b(4))) / b(0));
    fx = std::sqrt(lambda / b(0));
    fy = std::sqrt((lambda * b(0)) / (b(0) * b(2) - b(1) * b(1)));
    double skew = -1 * ((b(1) * fx * fx * fy) / lambda);
    cx = (skew * cy) / fy - (b(3) * fx * fx) / lambda;
  }

  void EstimatePoseFromHomography(Eigen::Matrix3d H, const Eigen::Matrix3d& K, colmap::Rigid3d& pose) {
    // The colmap 'PoseFromHomographyMatrix' uses a newer analytical solution but seems to use a different
    // convention for plane normal direction (unsure) which ends up giving a bad result for the pose translation.

    H = K.inverse() * H;
    H /= H.col(0).norm();  // roughly recover scale by normalizing H such that R column vec is unit length

    // The determinant of H must be > 0 to represent a rotation and not a reflection
    if (H.determinant() < 0) {
      H.array() *= -1.0;
    }

    Eigen::Vector3d t = H.col(2);

    Eigen::Matrix3d R;
    R.col(0) = H.col(0);
    R.col(1) = H.col(1);
    R.col(2) = H.col(0).cross(H.col(1));  // get R from H

    // "Fix" R to be proper rotation matrix using SVD.
    Eigen::JacobiSVD svd(R, Eigen::ComputeFullV | Eigen::ComputeFullU);
    R = svd.matrixU() * svd.matrixV().transpose();
    pose.rotation = Eigen::Quaterniond(R);
    pose.translation = t;
  }

  void CalibrateCamera(std::vector<std::vector<Eigen::Vector3d>>& object_points,
                       const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                       bool use_intrinsic_guess, std::vector<colmap::Rigid3d>& poses,
                       std::vector<double>* std_deviations_intrinsics) {
    if (camera.width <= 0 || camera.height <= 0 || camera.model_id == colmap::CameraModelId::kInvalid) {
      throw std::runtime_error("Camera width, height and model must be initialized!");
    }

    // create 2D plane points
    std::vector<std::vector<Eigen::Vector2d>> object_plane_points(object_points.size());
    for (size_t i = 0; i < object_points.size(); i++) {
      object_plane_points[i].reserve(object_points[i].size());
      for (size_t j = 0; j < object_points[i].size(); j++) {
        const auto& point_obj = object_points[i][j];
        object_plane_points[i].push_back({point_obj.x(), point_obj.y()});
      }
    }

    if (!use_intrinsic_guess) {
      double fx, fy, cx, cy;
      EstimateKFromHomographies(object_plane_points, image_points, fx, fy, cx, cy);
      camera.params = colmap::CameraModelInitializeParams(camera.model_id, (fx + fy) / 2, camera.width, camera.height);
    }

    const std::vector<std::vector<Eigen::Vector2d>>* undistorted_points_ptr;
    std::vector<std::vector<Eigen::Vector2d>> undistorted_points;
    undistorted_points_ptr = &undistorted_points;

    if (use_intrinsic_guess && !camera.IsUndistorted()) {
      UndistortImagePoints(image_points, camera, undistorted_points);
    }
    else {
      undistorted_points_ptr = &image_points;
    }

    ceres::Problem problem;
    poses.clear();
    poses.reserve(image_points.size());
    for (size_t i = 0; i < image_points.size(); i++) {
      std::vector<Eigen::Matrix3d> models;
      colmap::HomographyMatrixEstimator::Estimate(object_plane_points[i], (*undistorted_points_ptr)[i], &models);
      Eigen::Matrix3d H = models[0];

      colmap::Rigid3d pose;
      EstimatePoseFromHomography(H, camera.CalibrationMatrix(), pose);

      poses.push_back(pose);

      AddImageToProblem(problem, camera, poses[i], image_points[i], object_points[i]);
    }

    if (camera.model_id == colmap::PinholeCameraModel::model_id ||
        camera.model_id == colmap::SimplePinholeCameraModel::model_id) {
      // in case we have simple models keep the principal point constant
      std::vector<int> const_camera_params;
      const auto& params_idxs = camera.PrincipalPointIdxs();
      const_camera_params.insert(const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      colmap::SetSubsetManifold(static_cast<int>(camera.params.size()), const_camera_params, &problem, camera.params.data());
    }

    // Solve
    ceres::Solver::Options solver_options;

    solver_options.gradient_tolerance = 1e-20;
    solver_options.function_tolerance = 1e-20;
    solver_options.max_num_iterations = 100;
    solver_options.minimizer_progress_to_stdout = true;
    solver_options.linear_solver_type = ceres::DENSE_SCHUR;
    solver_options.num_threads = std::thread::hardware_concurrency();

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    std::string test = summary.FullReport();

    if (std_deviations_intrinsics) {
      std_deviations_intrinsics->clear();
      std_deviations_intrinsics->reserve(camera.params.size());

      ceres::Covariance::Options covariance_options;
      covariance_options.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
      ceres::Covariance covariance(covariance_options);
      if (covariance.Compute({camera.params.data()}, &problem)) {
        size_t num_params = camera.params.size();
        std::vector<double> covariance_mat(num_params * num_params);

        if (covariance.GetCovarianceBlock(camera.params.data(), camera.params.data(), covariance_mat.data())) {
          for (size_t i = 0; i < num_params; i++) {
            // diagonal indices
            size_t idx = i + i * num_params;
            double std = covariance_mat[idx] == 0 ? 0 : sqrt(covariance_mat[idx]);
            std_deviations_intrinsics->push_back(std);
          }
        }
      }
    }
  }

  double CalculateOverallRMS(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                             const std::vector<std::vector<Eigen::Vector2d>>& image_points,
                             const std::vector<colmap::Rigid3d>& poses, const colmap::Camera& camera,
                             std::vector<double>& per_view_rms) {
    CalculateperViewSquaredError(object_points, image_points, poses, camera, per_view_rms);

    double sum = 0;
    for (size_t i = 0; i < per_view_rms.size(); i++) {
      // first take the view squared error
      sum += per_view_rms[i];
      // and then replace it by the rms for the view
      per_view_rms[i] = std::sqrt(per_view_rms[i]);
    }

    return std::sqrt(sum / per_view_rms.size());
  }

  void CalculateperViewSquaredError(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                                    const std::vector<std::vector<Eigen::Vector2d>>& image_points,
                                    const std::vector<colmap::Rigid3d>& poses, const colmap::Camera& camera,
                                    std::vector<double>& per_view_se) {
    per_view_se.clear();
    per_view_se.reserve(object_points.size());
    for (size_t i = 0; i < object_points.size(); i++) {
      per_view_se.push_back(0);
      const colmap::Rigid3d& pose = poses[i];
      for (size_t j = 0; j < object_points[i].size(); j++) {
        const Eigen::Vector3d& point3D = object_points[i][j];
        const Eigen::Vector2d& image_point = image_points[i][j];

        per_view_se[i] +=
            colmap::CalculateSquaredReprojectionError(image_point, point3D, pose, camera, camera.IsCameraRefractive());
      }

      per_view_se[i] = per_view_se[i] / object_points[i].size();
    }
  }
}