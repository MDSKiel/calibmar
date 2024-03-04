#include "aruco_sift_extractor.h"
#include "calibmar/core/calibration_targets.h"

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

namespace {
  void ScaleMarkerCornersInPlace(std::vector<std::vector<cv::Point2f>>& markers, double scale_factor) {
    for (auto& marker : markers) {
      double mean_x = 0;
      double mean_y = 0;

      for (const auto& corner : marker) {
        mean_x += corner.x;
        mean_y += corner.y;
      }

      mean_x /= marker.size();
      mean_y /= marker.size();

      // to scale around the center, move mean to origin, scale and then move back to original location
      for (auto& corner : marker) {
        corner.x = (corner.x - mean_x) * scale_factor + mean_x;
        corner.y = (corner.y - mean_y) * scale_factor + mean_y;
      }
    }
  }

  bool PointInTriangle(const Eigen::Vector2d& point, const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
    // https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
    int as_x = point.x() - a.x;
    int as_y = point.y() - a.y;

    bool s_ab = (b.x - a.x) * as_y - (b.y - a.y) * as_x > 0;

    if ((c.x - a.x) * as_y - (c.y - a.y) * as_x > 0 == s_ab) {
      return false;
    }
    if ((c.x - b.x) * (point.y() - b.y) - (c.y - b.y) * (point.x() - b.x) > 0 != s_ab) {
      return false;
    }
    return true;
  }

  bool IsInsideAnyRectangle(std::vector<std::vector<cv::Point2f>> marker_corners, const Eigen::Vector2d& point) {
    for (const auto& marker : marker_corners) {
      bool is_inside = PointInTriangle(point, marker[0], marker[1], marker[2]);
      is_inside |= PointInTriangle(point, marker[0], marker[2], marker[3]);
      if (is_inside) {
        return true;
      }
    }
    return false;
  }

  void ExtractArucos(calibmar::Image& image, const calibmar::Pixmap& pixmap,
                     std::vector<std::vector<cv::Point2f>>& marker_corners, calibmar::ArucoMarkerTypes aruco_type) {
    cv::Mat image_data(pixmap.Data());
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = std::make_shared<cv::aruco::DetectorParameters>();
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(static_cast<int>(aruco_type)));
    cv::aruco::detectMarkers(image_data, dictionary, marker_corners, marker_ids, parameters, rejected_candidates);

    // store aruco keypoints in the image
    std::map<int, std::vector<Eigen::Vector2d>> aruco_keypoints;
    for (size_t i = 0; i < marker_ids.size(); i++) {
      std::vector<Eigen::Vector2d> corners;
      corners.reserve(marker_corners[i].size());

      for (const auto& corner : marker_corners[i]) {
        corners.push_back({corner.x, corner.y});
      }

      aruco_keypoints.emplace(marker_ids[i], corners);
    }
    image.SetArucoKeypoints(aruco_keypoints);
  };
}

namespace calibmar {
  ArucoSiftFeatureExtractor::ArucoSiftFeatureExtractor(const Options& options)
      : options_(options), sift_extractor_(SiftFeatureExtractor::Options{}) {}

  FeatureExtractor::Status ArucoSiftFeatureExtractor::Extract(Image& image, const Pixmap& pixmap) {
    if (!options_.only_extract_aruco) {
      return ExtractFull(image, pixmap);
    }

    std::vector<std::vector<cv::Point2f>> marker_corners;
    ExtractArucos(image, pixmap, marker_corners, options_.aruco_type);

    // To support livestream clibration the 2D points will also contain marker corners in the 'only_extract_aruco' case
    std::vector<Eigen::Vector2d> points_2D;
    points_2D.reserve(marker_corners.size() * 4);
    for (const auto& id_corners : image.ArucoKeypoints()) {
      for (const auto& corner : id_corners.second) {
        points_2D.push_back(corner);
      }
    }
    image.SetPoints2D(points_2D);

    return marker_corners.size() > 0 ? FeatureExtractor::Status::SUCCESS : FeatureExtractor::Status::DETECTION_ERROR;
  }

  FeatureExtractor::Status ArucoSiftFeatureExtractor::ExtractFull(Image& image, const Pixmap& pixmap) {
    cv::Mat image_data(pixmap.Data());

    std::vector<std::vector<cv::Point2f>> marker_corners;
    ExtractArucos(image, pixmap, marker_corners, options_.aruco_type);

    if (marker_corners.size() == 0) {
      return Status::DETECTION_ERROR;
    }

    // Scale up the aruco corners to generate the sift masking rectangles
    ScaleMarkerCornersInPlace(marker_corners, options_.masking_scale_factor);

    // Determine min/max x/y to only detect sift on a subimage
    float min_x = pixmap.Width();
    float max_x = 0;
    float min_y = pixmap.Height();
    float max_y = 0;
    for (const auto& marker : marker_corners) {
      for (const auto& corner : marker) {
        min_x = std::min(min_x, corner.x);
        max_x = std::max(max_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_y = std::max(max_y, corner.y);
      }
    }

    // limit to image
    min_x = std::max(0.0f, min_x);
    min_y = std::max(0.0f, min_y);
    max_x = std::min(static_cast<float>(pixmap.Width()), max_x);
    max_y = std::min(static_cast<float>(pixmap.Height()), max_y);

    int x_offset = static_cast<int>(min_x);
    int y_offset = static_cast<int>(min_y);
    int width = static_cast<int>((max_x)-x_offset);
    int height = static_cast<int>((max_y)-y_offset);

    Image sub_image;
    {
      // this constructor does not reallocate, but rather offers a subimage view into the original
      cv::Mat subview(image_data, cv::Rect(x_offset, y_offset, width, height));

      // extract SIFT in subimage
      Pixmap wrapper;
      wrapper.Assign(subview);
      if (sift_extractor_.Extract(sub_image, wrapper) != Status::SUCCESS) {
        return Status::DETECTION_ERROR;
      }
    }

    // Ensure detected points are also in randomly oriented mask rectangle
    std::vector<colmap::FeatureKeypoint> keypoints = sub_image.FeatureKeypoints();
    colmap::FeatureDescriptors descriptors = sub_image.FeatureDescriptors();
    std::vector<Eigen::Vector2d> points2D;
    points2D.reserve(sub_image.FeatureKeypoints().size());

    size_t out_index = 0;
    for (size_t i = 0; i < keypoints.size(); i++) {
      const colmap::FeatureKeypoint& keypoint = keypoints.at(i);
      Eigen::Vector2d point_translated{keypoint.x + x_offset, keypoint.y + y_offset};

      if (!IsInsideAnyRectangle(marker_corners, point_translated)) {
        // skip points outside of rectangle mask
      }
      else {
        // Retain this keypoint by copying it to the output index
        keypoints.at(out_index) =
            colmap::FeatureKeypoint(static_cast<float>(point_translated.x()), static_cast<float>(point_translated.y()),
                                    keypoint.a11, keypoint.a12, keypoint.a21, keypoint.a22);

        descriptors.row(out_index) = descriptors.row(i);
        out_index += 1;
        points2D.push_back(point_translated);
      }
    }

    keypoints.resize(out_index);
    descriptors.conservativeResize(out_index, descriptors.cols());

    if (points2D.size() == 0) {
      return Status::DETECTION_ERROR;
    }

    image.SetPoints2D(points2D);
    image.SetKeypoints(keypoints);
    image.SetDescriptors(descriptors);

    return Status::SUCCESS;
  }
}
