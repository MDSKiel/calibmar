#include "pose_suggestion.h"
#include "calibmar/core/camera_models.h"
#include "calibmar/pose_suggestion/simmulated_annealing.h"

#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <colmap/src/base/pose.h>
#include <colmap/src/base/projection.h>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace {
  // Polynomial from paper, that maps corner angle to uncertainty. According to paper this should also depend on average blur, but
  // the reference implementation only uses this
  double Angle2Corneruncertainty(double x) {
    return 1404.50760 * x * x - 49.2631560 * x * x * x + 0.94482 * x * x * x * x - 0.0093798 * x * x * x * x * x +
           0.0000455668 * x * x * x * x * x * x - 8.6160 * 1e-8 * x * x * x * x * x * x * x;
  }

  // x as euler angles and translation to quat and translation. Rotation order matches Calibration Wizard reference
  // implementation.
  void ToQuatAndVec(const std::vector<double>& x, Eigen::Vector4d& quat, Eigen::Vector3d& trans) {
    Eigen::Quaterniond quaternion = Eigen::AngleAxisd(x[0], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(x[1], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(x[2], Eigen::Vector3d::UnitX());
    quat << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
    trans << x[3], x[4], x[5];
  }

  int border_tolerance = 30;

  double CostFunctionWrapper(const std::vector<double>& x, void* f_data) {
    calibmar::pose_suggestion::CostFunctionData* data = static_cast<calibmar::pose_suggestion::CostFunctionData*>(f_data);

    return calibmar::pose_suggestion::CostFunction(x, data);
  }

  double TemperatureAnnealing(int i, std::vector<double>& temp, void* data) {
    std::pair<double, double>* bounds = static_cast<std::pair<double, double>*>(data);

    // since the temperature is used as the stepsize, start close to the bounds
    double T0_trans = bounds->first * 2;
    double T0_rot = bounds->second * 2;

    double max_T = 0;
    // temperature as T0 * 0.95^i, inspired by MATLAB simulannealbnd
    for (size_t j = 0; j < 3; j++) {
      temp[j] = T0_rot * std::pow(0.95, i);
      if (temp[j] > max_T) {
        max_T = temp[j];
      }
    }
    for (size_t j = 3; j < 6; j++) {
      temp[j] = T0_trans * std::pow(0.95, i);
      if (temp[j] > max_T) {
        max_T = temp[j];
      }
    }

    return max_T;
  }

  void CalculateInitialPose(const std::pair<double, double>& board_size, Eigen::Vector4d& next_rotation,
                            Eigen::Vector3d& next_translation, bool invert_y = false) {
    // center
    Eigen::Translation3d center(-board_size.first / 2, -board_size.second / 2, 0);
    // rotations
    // 90° z
    Eigen::AngleAxisd rotate_z(M_PI / 2, Eigen::Vector3d::UnitZ());
    // 45° x
    Eigen::AngleAxisd rotate_x(M_PI / 4, Eigen::Vector3d::UnitX());
    // 22.5° y
    Eigen::AngleAxisd rotate_y((invert_y ? -1 : 1) * M_PI / 8, Eigen::Vector3d::UnitY());
    // move in front of camera
    Eigen::Translation3d move_z(0, 0, 1.8 * board_size.first);

    Eigen::Affine3d t(center);
    t = move_z * rotate_y * rotate_x * rotate_z * t;

    Eigen::Quaterniond quat(t.rotation());
    Eigen::Vector3d trans(t.translation());

    next_rotation << quat.w(), quat.x(), quat.y(), quat.z();
    next_translation << trans.x(), trans.y(), trans.z();
  }

  void CalculateFullScreenPose(const colmap::Camera& camera, const std::pair<double, double>& board_size,
                               const std::pair<int, int>& image_size, const int columns, Eigen::Vector4d& next_rotation,
                               Eigen::Vector3d& next_translation) {
    // determine the amount of z to move the board so the projected points are close to image border
    // assuming a simple pinhole camera
    double pattern_width = board_size.first;
    double pattern_height = board_size.second;

    double x = image_size.first * 0.9;
    double y = image_size.second * 0.9;
    // flip width & height, because the pattern will be rotated 90°
    double zx = (pattern_height * camera.MeanFocalLength()) / x;
    double zy = (pattern_width * camera.MeanFocalLength()) / y;
    // higher z, so that the longer side is inside the image
    double z = std::max(zx, zy);

    // center
    Eigen::Translation3d center(-board_size.first / 2, -board_size.second / 2, 0);
    // 90° z
    Eigen::AngleAxisd rotate_z(M_PI / 2, Eigen::Vector3d::UnitZ());
    // move in front of camera
    Eigen::Translation3d move_z(0, 0, z);

    Eigen::Affine3d t(center);
    t = move_z * rotate_z * t;

    Eigen::Quaterniond quat(t.rotation());
    Eigen::Vector3d trans(t.translation());

    next_rotation << quat.w(), quat.x(), quat.y(), quat.z();
    next_translation << trans.x(), trans.y(), trans.z();
  }
}

namespace calibmar {
  namespace pose_suggestion {
    void ComputeJacobian(const std::vector<Image>& images, const std::vector<Eigen::Vector3d> points3D,
                         const colmap::Camera& camera, Eigen::MatrixXd& jacobian_intrinsics,
                         Eigen::MatrixXd& jacobian_extrinsics) {
      // TODO: computing the jacobian as Eigen::SparseMatrix<double> could speedup the cost function.
      // The jacobian parts are appended or appended to several times, which is tricky with SparseMatrix though.

      int num_points = points3D.size();
      int num_views = images.size();
      int height = 2 * num_points * num_views;

      if (jacobian_intrinsics.rows() < height || jacobian_intrinsics.cols() < camera.NumParams()) {
        jacobian_intrinsics.resize(height, camera.NumParams());
      }
      if (jacobian_extrinsics.rows() < height || jacobian_extrinsics.cols() < 6 * num_views) {
        jacobian_extrinsics.resize(height, 6 * num_views);
        jacobian_extrinsics.setZero();
      }

      Eigen::MatrixXd A, B;
      for (size_t i = 0; i < images.size(); i++) {
        const Eigen::Vector4d& rot = images[i].Rotation();
        const Eigen::Vector3d& trans = images[i].Translation();
        Eigen::Matrix3d rot_mat = Eigen::Quaterniond(rot(0), rot(1), rot(2), rot(3)).toRotationMatrix();
        ComputeJacobian(rot_mat, trans, points3D, camera, A, B);

        jacobian_intrinsics.block(i * num_points * 2, 0, A.rows(), A.cols()) = A;
        jacobian_extrinsics.block(i * num_points * 2, i * 6, B.rows(), B.cols()) = B;
      }
    }

    void ComputeJacobian(Eigen::Matrix3d& rot, const Eigen::Vector3d& trans, const std::vector<Eigen::Vector3d> points3D,
                         const colmap::Camera& camera, Eigen::MatrixXd& jacobian_intrinsics,
                         Eigen::MatrixXd& jacobian_extrinsics) {
      // always use 14 distortion params, but only fill the ones used by colmap
      // cv::projectPoints only accepts distortion coefficients in cv typical lengths (4,5,8,...,14)
      std::vector<double> distortion_params;
      for (size_t idx : camera.ExtraParamsIdxs()) {
        distortion_params.push_back(camera.Params()[idx]);
      }
      distortion_params.resize(14, 0);

      std::vector<cv::Point3f> points3D_cv;
      for (const Eigen::Vector3d& point : points3D) {
        points3D_cv.push_back(cv::Point3f(point.x(), point.y(), point.z()));
      }

      std::vector<cv::Point2f> image_points;
      cv::Mat jacobian_mat;
      cv::Mat camera_mat;
      cv::Mat rotation, translation;
      cv::eigen2cv(camera.CalibrationMatrix(), camera_mat);
      cv::eigen2cv(rot, rotation);
      cv::eigen2cv(trans, translation);
      bool fixed_aspect = camera.FocalLengthIdxs().size() == 1;

      cv::Mat jac_extrinsics, jac_intrinsics;
      if (CameraModel::IsFisheyeModel(camera.ModelId())) {
        // fisheye enforces rodrigues and 4 params
        cv::Mat rodrigues(1, 3, CV_64F);
        cv::Rodrigues(rotation, rodrigues);
        distortion_params.resize(4);
        cv::fisheye::projectPoints(points3D_cv, image_points, rodrigues, translation, camera_mat, distortion_params, 0,
                                   jacobian_mat);

        // extract the jacobian parts (ignore the skew component in last column)
        jac_intrinsics = jacobian_mat.colRange(0, camera.NumParams());
        jac_extrinsics = jacobian_mat.colRange(camera.NumParams(), camera.NumParams() + 6);
      }
      else {
        cv::projectPoints(points3D_cv, rotation, translation, camera_mat, distortion_params, image_points, jacobian_mat,
                          fixed_aspect);

        // extract the jacobian parts (extrinsics come before intrinsics, and one f is 0 if aspect ratio fixed)
        jac_extrinsics = jacobian_mat.colRange(0, 6);
        jac_intrinsics = jacobian_mat.colRange(6 + fixed_aspect, 6 + fixed_aspect + camera.NumParams());
      }
      Eigen::MatrixXd j;
      cv::cv2eigen(jacobian_mat, j);

      cv::cv2eigen(jac_intrinsics, jacobian_intrinsics);
      cv::cv2eigen(jac_extrinsics, jacobian_extrinsics);
    }

    void ComputeCornerUncertaintyAutoCorrMat(const std::vector<Eigen::Vector2d>& points2D, int views,
                                             const std::pair<int, int>& pattern_cols_rows, Eigen::SparseMatrix<double>& mat) {
      int width = pattern_cols_rows.first;
      int height = pattern_cols_rows.second;

      if (mat.rows() < width * height * 2 * views || mat.cols() < width * height * 2 * views) {
        throw std::runtime_error("Mat must have at least (pattern_width * pattern_height * 2 * views) rows and cols.");
      }

      // reserve 2 per column
      mat.reserve(width * height * 2 * views * 2);

      // for each point calculate the rotation and opening angle using its neighbours
      for (size_t view = 0; view < views; view++) {
        size_t view_offset = view * width * height;
        for (size_t y = 0; y < height; y++) {
          for (size_t x = 0; x < width; x++) {
            size_t index = view_offset + x + y * width;

            Eigen::Vector2d width_vec =
                x != width - 1 ? points2D[index + 1] - points2D[index] : points2D[index] - points2D[index - 1];

            Eigen::Vector2d height_vec =
                y != height - 1 ? points2D[index + width] - points2D[index] : points2D[index] - points2D[index - width];

            double angle = std::acos(width_vec.normalized().dot(height_vec.normalized())) * 180 / M_PI;

            Eigen::Vector2d v = angle > 90.0 ? (width_vec.normalized() + height_vec.normalized()).normalized()
                                             : (width_vec.normalized() - height_vec.normalized()).normalized();
            Eigen::Matrix2d R;
            R << v(0), -v(1), v(1), v(0);

            double s = 5e5;  // (empirical/magic) value from reference implementation
            Eigen::Matrix2d AC;
            AC << std::max(Angle2Corneruncertainty(180 - angle), Angle2Corneruncertainty(angle)) / s, 0, 0,
                std::min(Angle2Corneruncertainty(180 - angle), Angle2Corneruncertainty(angle)) / s;

            AC = R * AC * R.transpose();
            mat.insert(2 * index, 2 * index) = AC(0, 0);
            mat.insert(2 * index + 1, 2 * index) = AC(1, 0);
            mat.insert(2 * index + 1, 2 * index + 1) = AC(1, 1);
            mat.insert(2 * index, 2 * index + 1) = AC(0, 1);
          }
        }
      }
    }

    double CostFunction(const std::vector<double>& x, CostFunctionData* data) {
      const calibmar::Calibration& calibration = data->calibration;

      const colmap::Camera& camera = calibration.Camera();
      std::vector<Eigen::Vector3d> points3D;
      std::vector<Eigen::Vector2d> points2D;

      Eigen::Vector4d quat;
      Eigen::Vector3d translation;
      ToQuatAndVec(x, quat, translation);
      Eigen::Matrix3d rot = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).toRotationMatrix();
      Eigen::Affine3d trans = Eigen::Translation3d(translation) * Eigen::Affine3d(rot);
      for (const auto& [id, point] : calibration.Points3D()) {
        Eigen::Vector2d point_image = camera.WorldToImage((trans * point).hnormalized());
        // all points must lay inside image with border_tolerance
        if ((point_image.x() < border_tolerance || point_image.x() > camera.Width() - border_tolerance) ||
            ((point_image.y() < border_tolerance || point_image.y() > camera.Height() - border_tolerance))) {
          // additional constraint encoded as high cost
          // TODO: is there a better way to do this?
          return std::numeric_limits<double>::max();
        }

        points2D.push_back(point_image);
        points3D.push_back(point);
      }

      // Compute jacobian for proposed pose view
      Eigen::MatrixXd A_new;
      Eigen::MatrixXd B_new;

      ComputeJacobian(rot, Eigen::Vector3d(x[3], x[4], x[5]), points3D, camera, A_new, B_new);
      //  Append new jacobian view
      size_t n_views = calibration.Images().size();
      size_t n_points = calibration.Points3D().size();

      data->jacobian_extendable.block(n_views * n_points * 2, 0, A_new.rows(), A_new.cols()) = A_new;
      data->jacobian_extendable.block(n_views * n_points * 2, A_new.cols() + n_views * B_new.cols(), B_new.rows(), B_new.cols()) =
          B_new;

      Eigen::SparseMatrix<double> AC_current(data->pattern_cols_rows.first * data->pattern_cols_rows.second * 2,
                                             data->pattern_cols_rows.first * data->pattern_cols_rows.second * 2);
      calibmar::pose_suggestion::ComputeCornerUncertaintyAutoCorrMat(points2D, 1, data->pattern_cols_rows, AC_current);
      Eigen::SparseMatrix<double>& AC_extendable = data->autocorrelation_matrix_extendable;
      // insert the current ac into the overall one
      int size_ac = data->pattern_cols_rows.first * data->pattern_cols_rows.second * 2 * n_views;

      for (int k = 0; k < AC_current.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(AC_current, k); it; ++it) {
          AC_extendable.coeffRef(size_ac + it.row(), size_ac + it.col()) = it.value();
        }
      }

      Eigen::MatrixXd& J = data->jacobian_extendable;
      Eigen::MatrixXd M = J.transpose() * AC_extendable * J;

      int num_intrinsics = A_new.cols();
      // efficient coputation of upper left intrinsincs block of M.inverse() as shown in paper
      auto U = M.block(0, 0, num_intrinsics, num_intrinsics);
      auto W = M.block(0, num_intrinsics, num_intrinsics, M.cols() - num_intrinsics);
      auto V = M.block(num_intrinsics, num_intrinsics, M.rows() - num_intrinsics, M.cols() - num_intrinsics);
      auto F = (U - W * V.inverse() * W.transpose()).inverse();

      double trace = F.trace();

      // for the nlopt optimizer the function must not return NaN
      if (std::isnan(trace)) {
        trace = std::numeric_limits<double>::max();
      }

      return trace;
    }

    void EstimateNextBestPose(const Calibration& calibration, int columns_points, int rows_points, double square_size,
                              Eigen::Vector4d& next_rotation, Eigen::Vector3d& next_translation) {
      // origin is the first inner corner. Board size is actual pattern size -2 cols/ rows.
      std::pair<double, double> board_size((columns_points - 1) * square_size, (rows_points - 1) * square_size);
      std::pair<int, int> image_size(calibration.Camera().Width(), calibration.Camera().Height());

      if (calibration.Images().size() == 0) {
        // initial pose rotated centered
        CalculateInitialPose(board_size, next_rotation, next_translation);
        return;
      }
      else if (calibration.Images().size() == 1) {
        // second pose rotated centered, y rotation inverted
        CalculateInitialPose(board_size, next_rotation, next_translation, true);
        return;
      }
      else if (calibration.Images().size() == 2) {
        // third pose planar fullscreen
        CalculateFullScreenPose(calibration.Camera(), board_size, image_size, columns_points, next_rotation, next_translation);
        return;
      }

      // compute jacobian
      std::vector<Eigen::Vector3d> points3D;
      for (auto& [id, point] : calibration.Points3D()) {
        points3D.push_back(point);
      }

      size_t jac_extendable_views = (calibration.Images().size() + 1);
      size_t jac_extendable_rows = 2 * points3D.size() * jac_extendable_views;
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(jac_extendable_rows, calibration.Camera().NumParams());
      Eigen::MatrixXd B = Eigen::MatrixXd::Zero(jac_extendable_rows, jac_extendable_views * 6);
      pose_suggestion::ComputeJacobian(calibration.Images(), points3D, calibration.Camera(), A, B);
      Eigen::MatrixXd jacobian_extendable(jac_extendable_rows, A.cols() + B.cols());
      jacobian_extendable << A, B;

      // compute corner uncertainty autocorrelation matrix
      const colmap::Camera& camera = calibration.Camera();
      std::vector<Eigen::Vector2d> points2D;
      for (const calibmar::Image& image : calibration.Images()) {
        const Eigen::Vector4d& rot = image.Rotation();
        Eigen::Affine3d trans =
            Eigen::Quaterniond(rot(0), rot(1), rot(2), rot(3)).normalized() * Eigen::Translation3d(image.Translation());
        for (const auto& [idx, id] : image.Correspondences()) {
          const Eigen::Vector3d& point = calibration.Point3D(id);
          // TODO: Points are projected inside compute jacobian already, projecting here could be avoided.
          points2D.push_back(camera.WorldToImage((trans * point).hnormalized()));
        }
      }

      int views = calibration.Images().size();
      // autocorrelation matrix that can be extended in the cost function
      Eigen::SparseMatrix<double> AC_matrix_extendable(rows_points * columns_points * 2 * (views + 1),
                                                       rows_points * columns_points * 2 * (views + 1));
      std::pair<int, int> pattern_cols_rows(columns_points, rows_points);
      ComputeCornerUncertaintyAutoCorrMat(points2D, views, pattern_cols_rows, AC_matrix_extendable);

      double translation_bound = std::max(board_size.first, board_size.second) * 4;
      double rotation_bound = M_PI / 2;  // 90°
      std::vector<double> lb{-rotation_bound, -rotation_bound, -rotation_bound, -translation_bound, -translation_bound, 0};
      std::vector<double> ub{rotation_bound,    rotation_bound,    rotation_bound,
                             translation_bound, translation_bound, translation_bound};
      // std::vector<double> ub{M_PI/8, M_PI/8, M_PI/8, translation_bound, translation_bound, translation_bound};
      CostFunctionData data{calibration, jacobian_extendable, AC_matrix_extendable, pattern_cols_rows};
      double minf;
      // TODO: better initialization?
      std::vector<double> x{0, 0, 0, -board_size.first / 2, -board_size.first / 2, 2 * board_size.first};

      std::pair<double, double> temp_data = std::make_pair(translation_bound, rotation_bound);
      SimulatedAnnealing(x, minf, ub, lb, 1000, CostFunctionWrapper, &data, TemperatureAnnealing, &temp_data);

      ToQuatAndVec(x, next_rotation, next_translation);
    }
  }
}