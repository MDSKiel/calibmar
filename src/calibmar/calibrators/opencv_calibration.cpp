#include "opencv_calibration.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace {
  int TranslateToOpenCVFlags(colmap::CameraModelId colmap_modelid) {
    switch (colmap_modelid) {
      case colmap::SimplePinholeCameraModel::model_id:
        return cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
      case colmap::PinholeCameraModel::model_id:
        return cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
      case colmap::SimpleRadialCameraModel::model_id:
        return cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;
      case colmap::RadialCameraModel::model_id:
        return cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3;
      case colmap::OpenCVCameraModel::model_id:
        return cv::CALIB_FIX_K3;
      case colmap::FullOpenCVCameraModel::model_id:
        return cv::CALIB_RATIONAL_MODEL;
      case colmap::OpenCVFisheyeCameraModel::model_id:
        return cv::fisheye::CALIB_FIX_SKEW;
      default:
        throw std::runtime_error("Bad CameraModel");
    }
  }

  void GetCvParams(const colmap::Camera& camera, cv::Mat& camera_matrix, std::vector<double>& distortion_coefficients) {
    cv::eigen2cv(camera.CalibrationMatrix(), camera_matrix);

    // colmap ordering matches opencv ordering
    for (const size_t idx : camera.ExtraParamsIdxs()) {
      distortion_coefficients.push_back(camera.params.at(idx));
    }
  }

  void CvToColmapCamera(colmap::Camera& camera, const cv::Mat& camera_matrix,
                        const std::vector<double>& distortion_coefficients) {
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

    camera.params = params;
  }

  double CalibrateCameraCV(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                           const std::vector<std::vector<Eigen::Vector2d>>& image_points, colmap::Camera& camera,
                           bool use_intrinsic_guess, bool fast, std::vector<Eigen::Quaterniond*>& rotation_vecs,
                           std::vector<Eigen::Vector3d*>& translation_vecs, std::vector<double>& std_deviations_intrinsics,
                           std::vector<double>& std_deviations_extrinsics, std::vector<double>& per_view_rms) {
    if (camera.width <= 0 || camera.height <= 0 || camera.model_id == colmap::CameraModelId::kInvalid) {
      throw std::runtime_error("Camera width, height and model must be initialized!");
    }

    int flags = TranslateToOpenCVFlags(camera.model_id);

    cv::Mat camera_mat;
    if (camera.model_id == colmap::PinholeCameraModel::model_id ||
        camera.model_id == colmap::SimplePinholeCameraModel::model_id) {
      // For pinhole models without distortion, fix the principal point
      flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
    }

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
    // + 1 to cause principal point initialisation to the expected value
    cv::Size image_size(camera.width + 1, camera.height + 1);
    if (!calibmar::CameraModel::IsFisheyeModel(camera.model_id)) {
      rms = cv::calibrateCamera(pointSets3D, pointSets2D, image_size, camera_mat, distortion_coefficients, rotation_cv,
                                translation_cv, std_deviations_intrinsics, std_deviations_extrinsics, per_view_rms, flags);
    }
    else {
      rms = cv::fisheye::calibrate(pointSets3D, pointSets2D, image_size, camera_mat, distortion_coefficients, rotation_cv,
                                   translation_cv, flags);
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
    CvToColmapCamera(camera, camera_mat, distortion_coefficients);

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

    double StereoCalibrateCamera(const std::vector<std::vector<Eigen::Vector3d>>& object_points,
                                 const std::vector<std::vector<Eigen::Vector2d>>& image_points1,
                                 const std::vector<std::vector<Eigen::Vector2d>>& image_points2, colmap::Camera& camera1,
                                 colmap::Camera& camera2, colmap::Rigid3d& pose, bool use_intrinsic_guess, bool fix_intrinsic,
                                 bool same_focal_length, std::vector<std::vector<double>>& per_view_rms) {
      if (object_points.size() != image_points1.size() || object_points.size() != image_points2.size()) {
        throw std::runtime_error("Image and objects points must match for stereo calibration!");
      }

      if (camera1.model_id != camera2.model_id) {
        throw std::runtime_error("Camera models must be of same type for stereo calibration!");
      }

      if (fix_intrinsic && !use_intrinsic_guess) {
        throw std::runtime_error("Can not fix intrinsics if no intrinsics given!");
      }

      std::vector<std::vector<cv::Point3f>> pointSets3D(object_points.size());
      std::vector<std::vector<cv::Point2f>> pointSets2D_1(image_points1.size());
      std::vector<std::vector<cv::Point2f>> pointSets2D_2(image_points2.size());
      for (size_t i = 0; i < object_points.size(); i++) {
        int set_size = object_points[i].size();
        for (size_t j = 0; j < set_size; j++) {
          pointSets3D[i].push_back(cv::Point3f(object_points[i][j].x(), object_points[i][j].y(), object_points[i][j].z()));
          pointSets2D_1[i].push_back(cv::Point2f(image_points1[i][j].x(), image_points1[i][j].y()));
          pointSets2D_2[i].push_back(cv::Point2f(image_points2[i][j].x(), image_points2[i][j].y()));
        }
      }

      int flags = TranslateToOpenCVFlags(camera1.model_id);

      if (same_focal_length) {
        flags |= cv::CALIB_SAME_FOCAL_LENGTH;
      }

      cv::Mat camera_mat1;
      cv::Mat camera_mat2;
      std::vector<double> distortion_coefficients1;
      std::vector<double> distortion_coefficients2;

      if (camera1.model_id == colmap::PinholeCameraModel::model_id ||
          camera1.model_id == colmap::SimplePinholeCameraModel::model_id) {
        // For pinhole models without distortion, fix the principal point
        flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
      }

      if (use_intrinsic_guess) {
        // Careful: if CALIB_FIX_ASPECT_RATIO is used the camera_matrix should either be empty or set to valid values.
        // Opencv will use the aspect ratio of the focal lengths. If they are 0 (e.g. not set), the calibration will return NAN.
        flags |= cv::CALIB_USE_INTRINSIC_GUESS;

        if (fix_intrinsic) {
          flags |= cv::CALIB_FIX_INTRINSIC;
        }

        GetCvParams(camera1, camera_mat1, distortion_coefficients1);
        GetCvParams(camera2, camera_mat2, distortion_coefficients2);
      }

      cv::Mat R, T, perViewError;
      double rms;
      // + 1 to cause principal point initialisation to the expected value
      cv::Size image_size(camera1.width + 1, camera1.height + 1);
      if (!calibmar::CameraModel::IsFisheyeModel(camera1.model_id)) {
        rms = cv::stereoCalibrate(pointSets3D, pointSets2D_1, pointSets2D_2, camera_mat1, distortion_coefficients1, camera_mat2,
                                  distortion_coefficients2, image_size, R, T, cv::noArray(), cv::noArray(), perViewError, flags);
      }
      else {
        rms = cv::fisheye::stereoCalibrate(pointSets3D, pointSets2D_1, pointSets2D_2, camera_mat1, distortion_coefficients1,
                                           camera_mat2, distortion_coefficients2, image_size, R, T, flags);
      }

      // Convert to colmap cameras
      CvToColmapCamera(camera1, camera_mat1, distortion_coefficients1);
      CvToColmapCamera(camera2, camera_mat2, distortion_coefficients2);

      // Convert pose info
      Eigen::Matrix3d rot;
      cv::cv2eigen(R, rot);
      pose.translation = Eigen::Vector3d(T.at<double>(0), T.at<double>(1), T.at<double>(2));
      pose.rotation = Eigen::Quaterniond(rot);

      // Convert per view RMS
      per_view_rms.push_back(perViewError.col(0));
      per_view_rms.push_back(perViewError.col(1));

      return rms;
    }
  }
}