#include "opencv_calibration.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace {
  int TranslateToOpenCVFlags(int colmap_modelid) {
    switch (colmap_modelid) {
      case colmap::SimplePinholeCameraModel::kModelId:
        return cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
      case colmap::PinholeCameraModel::kModelId:
        return cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
      case colmap::SimpleRadialCameraModel::kModelId:
        return cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
      case colmap::RadialCameraModel::kModelId:
        return cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3;
      case colmap::OpenCVCameraModel::kModelId:
        return cv::CALIB_FIX_K3;
      case colmap::FullOpenCVCameraModel::kModelId:
        return cv::CALIB_RATIONAL_MODEL;
      case colmap::OpenCVFisheyeCameraModel::kModelId:
        return cv::fisheye::CALIB_FIX_SKEW;
      default:
        throw std::runtime_error("Bad CameraModel");
    }
  }

  void GetCvParams(const colmap::Camera& camera, cv::Mat& camera_matrix, std::vector<double>& distortion_coefficients) {
    cv::eigen2cv(camera.CalibrationMatrix(), camera_matrix);

    // colmap ordering matches opencv ordering
    for (const size_t idx : camera.ExtraParamsIdxs()) {
      distortion_coefficients.push_back(camera.Params().at(idx));
    }
  }

  void SetParams(colmap::Camera& camera, const cv::Mat& camera_matrix, const std::vector<double>& distortion_coefficients) {
    std::vector<double> params;

    if (camera.FocalLengthIdxs().size() == 1) {
      params.push_back(camera_matrix.at<double>(0, 0));  // f
    }
    else {
      params.push_back(camera_matrix.at<double>(0, 0));  // fx
      params.push_back(camera_matrix.at<double>(1, 1));  // fy
    }
    params.push_back(camera_matrix.at<double>(0, 2));  // cx
    params.push_back(camera_matrix.at<double>(1, 2));  // cy

    // colmap ordering matches opencv ordering
    for (size_t i = 0; i < camera.ExtraParamsIdxs().size(); i++) {
      params.push_back(distortion_coefficients.at(i));
    }

    camera.SetParams(params);
  }

  double CalibrateCameraCV(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Quaterniond*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs, std::vector<double>& std_deviations_intrinsics,
                           std::vector<double>& std_deviations_extrinsics, std::vector<double>& per_view_rms) {
    int flags = TranslateToOpenCVFlags(camera.ModelId());

    cv::Mat camera_mat;
    std::vector<double> distortion_coefficients;
    if (use_intrinsic_guess) {
      // Careful: if CALIB_FIX_ASPECT_RATIO is used the camera_matrix should either be empty or set to valid values.
      // Opencv will use the aspect ratio of the focal lengths. If they are 0 (e.g. not set), the calibration will return NAN.
      flags |= cv::CALIB_USE_INTRINSIC_GUESS;
      GetCvParams(camera, camera_mat, distortion_coefficients);
    }
    if (fast) {
      flags |= cv::CALIB_USE_LU;
    }

    std::vector<std::vector<cv::Point3f>> pointSets3D(object_points.size());
    std::vector<std::vector<cv::Point2f>> pointSets2D(image_points.size());
    for (size_t i = 0; i < object_points.size(); i++) {
      int set_size = object_points[i].size();
      for (size_t j = 0; j < set_size; j++) {
        pointSets3D[i].push_back(cv::Point3f(object_points[i][j].x(), object_points[i][j].y(), object_points[i][j].z()));
        pointSets2D[i].push_back(cv::Point2f(image_points[i][j].x(), image_points[i][j].y()));
      }
    }

    std::vector<cv::Mat> rotation_cv, translation_cv;
    double rms;
    if (!calibmar::CameraModel::IsFisheyeModel(camera.ModelId())) {
      rms = cv::calibrateCamera(pointSets3D, pointSets2D, cv::Size(camera.Width(), camera.Height()), camera_mat,
                                distortion_coefficients, rotation_cv, translation_cv, std_deviations_intrinsics,
                                std_deviations_extrinsics, per_view_rms, flags);
    }
    else {
      rms = cv::fisheye::calibrate(pointSets3D, pointSets2D, cv::Size(camera.Width(), camera.Height()), camera_mat,
                                   distortion_coefficients, rotation_cv, translation_cv, flags);
    }

    // convert the cv angle axis representation to quat vector.
    for (size_t i = 0; i < rotation_vecs.size(); i++) {
      Eigen::Vector3d rotation;
      cv::cv2eigen(rotation_cv[i], rotation);
      Eigen::Quaterniond quat(Eigen::AngleAxisd(rotation.norm(), rotation.normalized()));
      *rotation_vecs[i] = quat;

      Eigen::Vector3d translation;
      cv::cv2eigen(translation_cv[i], translation);
      *translation_vecs[i] = translation;
    }

    // Assign camera parameters
    SetParams(camera, camera_mat, distortion_coefficients);

    return rms;
  }
}

namespace calibmar {
  namespace opencv_calibration {

    double CalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Quaterniond*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs, std::vector<double>& std_deviations_intrinsics,
                           std::vector<double>& std_deviations_extrinsics, std::vector<double>& per_view_rms) {
      return CalibrateCameraCV(object_points, image_points, camera, use_intrinsic_guess, fast, rotation_vecs, translation_vecs,
                               std_deviations_intrinsics, std_deviations_extrinsics, per_view_rms);
    }

    double CalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Quaterniond*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs) {
      std::vector<double> std_deviations_intrinsics, std_deviations_extrinsics, per_view_rms;
      return CalibrateCameraCV(object_points, image_points, camera, use_intrinsic_guess, fast, rotation_vecs, translation_vecs,
                               std_deviations_intrinsics, std_deviations_extrinsics, per_view_rms);
    }
  }
}