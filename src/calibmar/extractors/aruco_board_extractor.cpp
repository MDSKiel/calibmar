#include "calibmar/extractors/aruco_board_extractor.h"

namespace {
  bool CloseToBorder(const cv::Point2f& corner, int min_distance, int image_width, int image_height) {
    return corner.x < min_distance || corner.y < min_distance || corner.x > image_width - min_distance ||
           corner.y > image_height - min_distance;
  }

}

namespace calibmar {
  ArucoBoardFeatureExtractor::ArucoBoardFeatureExtractor(const Options& options) : options_(options) {
    // 4 corners per marker
    int size = options.marker_cols * options.marker_rows;

    int width = options.marker_cols;
    int height = options.marker_rows;

    for (size_t row = 0; row < options_.marker_rows; row++) {
      for (size_t col = 0; col < options_.marker_cols; col++) {
        double x = col * (options_.marker_size + options.marker_spacing);
        double y = row * (options_.marker_size + options.marker_spacing);

        int marker_col, marker_row;
        switch (options.grid_origin) {
          case ArucoGridOrigin::TopLeft:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? col : row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? row : col;
            break;
          case ArucoGridOrigin::TopRight:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? (width - 1) - col : row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? row : (width - 1) - col;
            break;
          case ArucoGridOrigin::BottomLeft:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? col : (height - 1) - row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? (height - 1) - row : col;
            break;
          case ArucoGridOrigin::BottomRight:
            marker_col = options.grid_direction == ArucoGridDirection::Horizontal ? (width - 1) - col : (height - 1) - row;
            marker_row = options.grid_direction == ArucoGridDirection::Horizontal ? (height - 1) - row : (width - 1) - col;
            break;
        }
        int marker_id = options.start_id + marker_col +
                        marker_row * (options.grid_direction == ArucoGridDirection::Horizontal ? width : height);

        points3D_[MapMarkerCornerToPointId(marker_id, 0)] = Eigen::Vector3d(x, y, 0);
        points3D_[MapMarkerCornerToPointId(marker_id, 1)] = Eigen::Vector3d(x + options_.marker_size, y, 0);
        points3D_[MapMarkerCornerToPointId(marker_id, 2)] =
            Eigen::Vector3d(x + options_.marker_size, y + options_.marker_size, 0);
        points3D_[MapMarkerCornerToPointId(marker_id, 3)] = Eigen::Vector3d(x, y + options_.marker_size, 0);
      }
    }
  }

  FeatureExtractor::Status ArucoBoardFeatureExtractor::Extract(Image& image, const Pixmap& pixmap) {
    if (pixmap.Width() <= 0 || pixmap.Height() <= 0) {
      return Status::DETECTION_ERROR;
    }

    bool is_apriltag = options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_16h5 ||
                       options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_25h9 ||
                       options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_36h10 ||
                       options_.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_36h11;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Mat image_data(pixmap.Data());
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = std::make_shared<cv::aruco::DetectorParameters>();
    parameters->markerBorderBits = options_.border_bits;
    parameters->maxErroneousBitsInBorderRate = 0.0;
    // this does currently not seem to work as intended with opencv so this is implemented here
    parameters->minDistanceToBorder = 0;
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    if (is_apriltag) {
      // The opencv implementation of AprilTag seems to work differently
      parameters->adaptiveThreshWinSizeStep = 1;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(static_cast<int>(options_.aruco_type)));
    cv::aruco::detectMarkers(image_data, dictionary, marker_corners, marker_ids, parameters, rejected_candidates);

    if (marker_corners.size() == 0) {
      return Status::DETECTION_ERROR;
    }

    std::map<int, size_t> id_to_idx;
    for (size_t i = 0; i < marker_ids.size(); i++) {
      // sort the points accoring to marker id, to keep stable detection for livestream
      id_to_idx[marker_ids[i]] = i;
    }

    // store aruco keypoints in the image
    image.SetPoints2D({});
    image.ClearCorrespondences();
    std::map<int, std::vector<Eigen::Vector2d>> aruco_keypoints;

    int min_distance_to_border = (int)(std::min(pixmap.Width(), pixmap.Height()) * 0.008);
    int max_id = options_.marker_cols * options_.marker_rows + options_.start_id - 1;
    for (const auto& marker_id_idx : id_to_idx) {
      if (marker_id_idx.first > max_id || marker_id_idx.first < options_.start_id) {
        // sanity check to prevent detections we know to be wrong
        continue;
      }
      size_t i = marker_id_idx.second;

      bool close_to_border = false;
      for (const auto& corner : marker_corners[i]) {
        if (CloseToBorder(corner, min_distance_to_border, pixmap.Width(), pixmap.Height())) {
          close_to_border = true;
          break;
        }
      }
      if (close_to_border) {
        continue;
      }

      std::vector<Eigen::Vector2d> corners;
      corners.reserve(marker_corners[i].size());
      for (size_t j = 0; j < marker_corners[i].size(); j++) {
        // for apriltag the opencv detector sets the origin to the bottom right,
        // which is inconsistent with Aruco, so shift them to match top left origin
        size_t j2 = is_apriltag ? (j + 2) % 4 : j;  // 2, 3, 0, 1 : 0, 1, 2, 3
        const auto& corner = marker_corners[i][j2];
        // OpenCV usually uses the convetion of 0,0 being the pixel center, while colmap uses 0.5,0.5 as center
        Eigen::Vector2d point2D(corner.x + 0.5, corner.y + 0.5);
        corners.push_back(point2D);
        size_t idx = image.AddPoint2D(point2D);
        image.SetPoint3DforPoint2D(MapMarkerCornerToPointId(marker_ids[i], j), idx);
      }

      aruco_keypoints.emplace(marker_ids[i], corners);
    }

    if (image.Points2D().size() == 0) {
      return Status::DETECTION_ERROR;
    }

    image.SetArucoKeypoints(aruco_keypoints);

    return Status::SUCCESS;
  }

  int ArucoBoardFeatureExtractor::MapMarkerCornerToPointId(int marker_id, int corner_idx) {
    return (marker_id - options_.start_id) * 4 + corner_idx;
  }
}
