// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "colmap/estimators/two_view_geometry.h"

#include "colmap/estimators/cost_functions.h"
#include "colmap/estimators/essential_matrix.h"
#include "colmap/estimators/fundamental_matrix.h"
#include "colmap/estimators/generalized_relative_pose.h"
#include "colmap/estimators/homography_matrix.h"
#include "colmap/estimators/refrac_relative_pose.h"
#include "colmap/estimators/translation_transform.h"
#include "colmap/geometry/essential_matrix.h"
#include "colmap/geometry/homography_matrix.h"
#include "colmap/geometry/pose.h"
#include "colmap/geometry/triangulation.h"
#include "colmap/math/random.h"
#include "colmap/optim/loransac.h"
#include "colmap/optim/ransac.h"
#include "colmap/scene/camera.h"
#include "colmap/scene/projection.h"

#include <unordered_set>

namespace colmap {
namespace {

FeatureMatches ExtractInlierMatches(const FeatureMatches& matches,
                                    const size_t num_inliers,
                                    const std::vector<char>& inlier_mask) {
  FeatureMatches inlier_matches(num_inliers);
  size_t j = 0;
  for (size_t i = 0; i < matches.size(); ++i) {
    if (inlier_mask[i]) {
      inlier_matches[j] = matches[i];
      j += 1;
    }
  }
  return inlier_matches;
}

FeatureMatches ExtractOutlierMatches(const FeatureMatches& matches,
                                     const FeatureMatches& inlier_matches) {
  CHECK_GE(matches.size(), inlier_matches.size());

  std::unordered_set<std::pair<point2D_t, point2D_t>> inlier_matches_set;
  inlier_matches_set.reserve(inlier_matches.size());
  for (const auto& match : inlier_matches) {
    inlier_matches_set.emplace(match.point2D_idx1, match.point2D_idx2);
  }

  FeatureMatches outlier_matches;
  outlier_matches.reserve(matches.size() - inlier_matches.size());

  for (const auto& match : matches) {
    if (inlier_matches_set.count(
            std::make_pair(match.point2D_idx1, match.point2D_idx2)) == 0) {
      outlier_matches.push_back(match);
    }
  }

  return outlier_matches;
}

inline bool IsImagePointInBoundingBox(const Eigen::Vector2d& point,
                                      const double minx,
                                      const double maxx,
                                      const double miny,
                                      const double maxy) {
  return point.x() >= minx && point.x() <= maxx && point.y() >= miny &&
         point.y() <= maxy;
}

TwoViewGeometry EstimateCalibratedHomography(
    const Camera& camera1,
    const std::vector<Eigen::Vector2d>& points1,
    const Camera& camera2,
    const std::vector<Eigen::Vector2d>& points2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options) {
  TwoViewGeometry geometry;

  const size_t min_num_inliers = static_cast<size_t>(options.min_num_inliers);
  if (matches.size() < min_num_inliers) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  // Extract corresponding points.
  std::vector<Eigen::Vector2d> matched_points1(matches.size());
  std::vector<Eigen::Vector2d> matched_points2(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    matched_points1[i] = points1[matches[i].point2D_idx1];
    matched_points2[i] = points2[matches[i].point2D_idx2];
  }

  // Estimate planar or panoramic model.

  LORANSAC<HomographyMatrixEstimator, HomographyMatrixEstimator> H_ransac(
      options.ransac_options);
  const auto H_report = H_ransac.Estimate(matched_points1, matched_points2);
  geometry.H = H_report.model;

  if (!H_report.success || H_report.support.num_inliers < min_num_inliers) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  } else {
    geometry.config = TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC;
  }

  geometry.inlier_matches = ExtractInlierMatches(
      matches, H_report.support.num_inliers, H_report.inlier_mask);
  if (options.detect_watermark && DetectWatermark(camera1,
                                                  matched_points1,
                                                  camera2,
                                                  matched_points2,
                                                  H_report.support.num_inliers,
                                                  H_report.inlier_mask,
                                                  options)) {
    geometry.config = TwoViewGeometry::ConfigurationType::WATERMARK;
  }

  if (options.compute_relative_pose) {
    EstimateTwoViewGeometryPose(camera1, points1, camera2, points2, &geometry);
  }

  return geometry;
}

TwoViewGeometry EstimateUncalibratedTwoViewGeometry(
    const Camera& camera1,
    const std::vector<Eigen::Vector2d>& points1,
    const Camera& camera2,
    const std::vector<Eigen::Vector2d>& points2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options) {
  TwoViewGeometry geometry;

  const size_t min_num_inliers = static_cast<size_t>(options.min_num_inliers);
  if (matches.size() < static_cast<size_t>(min_num_inliers)) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  // Extract corresponding points.
  std::vector<Eigen::Vector2d> matched_points1(matches.size());
  std::vector<Eigen::Vector2d> matched_points2(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    matched_points1[i] = points1[matches[i].point2D_idx1];
    matched_points2[i] = points2[matches[i].point2D_idx2];
  }

  // Estimate epipolar model.

  LORANSAC<FundamentalMatrixSevenPointEstimator,
           FundamentalMatrixEightPointEstimator>
      F_ransac(options.ransac_options);
  const auto F_report = F_ransac.Estimate(matched_points1, matched_points2);
  geometry.F = F_report.model;

  // Estimate planar or panoramic model.

  LORANSAC<HomographyMatrixEstimator, HomographyMatrixEstimator> H_ransac(
      options.ransac_options);
  const auto H_report = H_ransac.Estimate(matched_points1, matched_points2);
  geometry.H = H_report.model;

  if ((!F_report.success && !H_report.success) ||
      (F_report.support.num_inliers < min_num_inliers &&
       H_report.support.num_inliers < min_num_inliers)) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  // Determine inlier ratios of different models.

  const double H_F_inlier_ratio =
      static_cast<double>(H_report.support.num_inliers) /
      F_report.support.num_inliers;

  const std::vector<char>* best_inlier_mask = &F_report.inlier_mask;
  int num_inliers = F_report.support.num_inliers;
  if (H_F_inlier_ratio > options.max_H_inlier_ratio) {
    geometry.config = TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC;
    if (H_report.support.num_inliers >= F_report.support.num_inliers) {
      num_inliers = H_report.support.num_inliers;
      best_inlier_mask = &H_report.inlier_mask;
    }
  } else {
    geometry.config = TwoViewGeometry::ConfigurationType::UNCALIBRATED;
  }

  geometry.inlier_matches =
      ExtractInlierMatches(matches, num_inliers, *best_inlier_mask);

  if (options.detect_watermark && DetectWatermark(camera1,
                                                  matched_points1,
                                                  camera2,
                                                  matched_points2,
                                                  num_inliers,
                                                  *best_inlier_mask,
                                                  options)) {
    geometry.config = TwoViewGeometry::ConfigurationType::WATERMARK;
  }

  if (options.compute_relative_pose) {
    EstimateTwoViewGeometryPose(camera1, points1, camera2, points2, &geometry);
  }

  return geometry;
}

TwoViewGeometry EstimateMultipleTwoViewGeometries(
    const Camera& camera1,
    const std::vector<Eigen::Vector2d>& points1,
    const Camera& camera2,
    const std::vector<Eigen::Vector2d>& points2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options) {
  FeatureMatches remaining_matches = matches;
  TwoViewGeometry multi_geometry;
  std::vector<TwoViewGeometry> geometries;
  TwoViewGeometryOptions options_copy = options;
  // Set to false to prevent recursive calls to this function.
  options_copy.multiple_models = false;
  while (true) {
    TwoViewGeometry geometry = EstimateTwoViewGeometry(
        camera1, points1, camera2, points2, remaining_matches, options_copy);
    if (geometry.config == TwoViewGeometry::ConfigurationType::DEGENERATE) {
      break;
    }

    if (options.multiple_ignore_watermark) {
      if (geometry.config != TwoViewGeometry::ConfigurationType::WATERMARK) {
        geometries.push_back(geometry);
      }
    } else {
      geometries.push_back(geometry);
    }

    remaining_matches =
        ExtractOutlierMatches(remaining_matches, geometry.inlier_matches);
  }

  if (geometries.empty()) {
    multi_geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
  } else if (geometries.size() == 1) {
    multi_geometry = geometries[0];
  } else {
    multi_geometry.config = TwoViewGeometry::ConfigurationType::MULTIPLE;
    for (const auto& geometry : geometries) {
      multi_geometry.inlier_matches.insert(multi_geometry.inlier_matches.end(),
                                           geometry.inlier_matches.begin(),
                                           geometry.inlier_matches.end());
    }
  }

  return multi_geometry;
}

}  // namespace

bool TwoViewGeometryOptions::Check() const {
  CHECK_OPTION_GE(min_num_inliers, 0);
  CHECK_OPTION_GE(min_E_F_inlier_ratio, 0);
  CHECK_OPTION_LE(min_E_F_inlier_ratio, 1);
  CHECK_OPTION_GE(max_H_inlier_ratio, 0);
  CHECK_OPTION_LE(max_H_inlier_ratio, 1);
  CHECK_OPTION_GE(watermark_min_inlier_ratio, 0);
  CHECK_OPTION_LE(watermark_min_inlier_ratio, 1);
  CHECK_OPTION_GE(watermark_border_size, 0);
  CHECK_OPTION_LE(watermark_border_size, 1);
  CHECK_OPTION_GT(ransac_options.max_error, 0);
  CHECK_OPTION_GE(ransac_options.min_inlier_ratio, 0);
  CHECK_OPTION_LE(ransac_options.min_inlier_ratio, 1);
  CHECK_OPTION_GE(ransac_options.confidence, 0);
  CHECK_OPTION_LE(ransac_options.confidence, 1);
  CHECK_OPTION_LE(ransac_options.min_num_trials, ransac_options.max_num_trials);
  return true;
}

TwoViewGeometry EstimateTwoViewGeometry(
    const Camera& camera1,
    const std::vector<Eigen::Vector2d>& points1,
    const Camera& camera2,
    const std::vector<Eigen::Vector2d>& points2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options) {
  if (options.multiple_models) {
    return EstimateMultipleTwoViewGeometries(
        camera1, points1, camera2, points2, matches, options);
  } else if (options.force_H_use) {
    return EstimateCalibratedHomography(
        camera1, points1, camera2, points2, matches, options);
  } else if (camera1.has_prior_focal_length && camera2.has_prior_focal_length) {
    return EstimateCalibratedTwoViewGeometry(
        camera1, points1, camera2, points2, matches, options);
  } else {
    return EstimateUncalibratedTwoViewGeometry(
        camera1, points1, camera2, points2, matches, options);
  }
}

bool EstimateTwoViewGeometryPose(const Camera& camera1,
                                 const std::vector<Eigen::Vector2d>& points1,
                                 const Camera& camera2,
                                 const std::vector<Eigen::Vector2d>& points2,
                                 TwoViewGeometry* geometry) {
  // We need a valid epopolar geometry to estimate the relative pose.
  if (geometry->config != TwoViewGeometry::ConfigurationType::CALIBRATED &&
      geometry->config != TwoViewGeometry::ConfigurationType::UNCALIBRATED &&
      geometry->config != TwoViewGeometry::ConfigurationType::PLANAR &&
      geometry->config != TwoViewGeometry::ConfigurationType::PANORAMIC &&
      geometry->config !=
          TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC) {
    return false;
  }

  // Extract normalized inlier points.
  std::vector<Eigen::Vector2d> inlier_points1_normalized;
  inlier_points1_normalized.reserve(geometry->inlier_matches.size());
  std::vector<Eigen::Vector2d> inlier_points2_normalized;
  inlier_points2_normalized.reserve(geometry->inlier_matches.size());
  for (const auto& match : geometry->inlier_matches) {
    inlier_points1_normalized.push_back(
        camera1.CamFromImg(points1[match.point2D_idx1]));
    inlier_points2_normalized.push_back(
        camera2.CamFromImg(points2[match.point2D_idx2]));
  }

  Eigen::Matrix3d cam2_from_cam1_rot_mat;
  std::vector<Eigen::Vector3d> points3D;

  if (geometry->config == TwoViewGeometry::ConfigurationType::CALIBRATED ||
      geometry->config == TwoViewGeometry::ConfigurationType::UNCALIBRATED) {
    // Try to recover relative pose for calibrated and uncalibrated
    // configurations. In the uncalibrated case, this most likely leads to a
    // ill-defined reconstruction, but sometimes it succeeds anyways after e.g.
    // subsequent bundle-adjustment etc.
    PoseFromEssentialMatrix(geometry->E,
                            inlier_points1_normalized,
                            inlier_points2_normalized,
                            &cam2_from_cam1_rot_mat,
                            &geometry->cam2_from_cam1.translation,
                            &points3D);
  } else if (geometry->config == TwoViewGeometry::ConfigurationType::PLANAR ||
             geometry->config ==
                 TwoViewGeometry::ConfigurationType::PANORAMIC ||
             geometry->config ==
                 TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC) {
    Eigen::Vector3d normal;
    PoseFromHomographyMatrix(geometry->H,
                             camera1.CalibrationMatrix(),
                             camera2.CalibrationMatrix(),
                             inlier_points1_normalized,
                             inlier_points2_normalized,
                             &cam2_from_cam1_rot_mat,
                             &geometry->cam2_from_cam1.translation,
                             &normal,
                             &points3D);
  } else {
    return false;
  }

  geometry->cam2_from_cam1.rotation =
      Eigen::Quaterniond(cam2_from_cam1_rot_mat);

  if (points3D.empty()) {
    geometry->tri_angle = 0;
  } else {
    geometry->tri_angle = Median(
        CalculateTriangulationAngles(Eigen::Vector3d::Zero(),
                                     -cam2_from_cam1_rot_mat.transpose() *
                                         geometry->cam2_from_cam1.translation,
                                     points3D));
  }

  if (geometry->config ==
      TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC) {
    if (geometry->cam2_from_cam1.translation.norm() == 0) {
      geometry->config = TwoViewGeometry::ConfigurationType::PANORAMIC;
      geometry->tri_angle = 0;
    } else {
      geometry->config = TwoViewGeometry::ConfigurationType::PLANAR;
    }
  }

  return true;
}

TwoViewGeometry EstimateCalibratedTwoViewGeometry(
    const Camera& camera1,
    const std::vector<Eigen::Vector2d>& points1,
    const Camera& camera2,
    const std::vector<Eigen::Vector2d>& points2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options) {
  CHECK(options.Check());

  TwoViewGeometry geometry;

  const size_t min_num_inliers = static_cast<size_t>(options.min_num_inliers);
  if (matches.size() < min_num_inliers) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  // Extract corresponding points.
  std::vector<Eigen::Vector2d> matched_points1(matches.size());
  std::vector<Eigen::Vector2d> matched_points2(matches.size());
  std::vector<Eigen::Vector2d> matched_points1_normalized(matches.size());
  std::vector<Eigen::Vector2d> matched_points2_normalized(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    const point2D_t idx1 = matches[i].point2D_idx1;
    const point2D_t idx2 = matches[i].point2D_idx2;
    matched_points1[i] = points1[idx1];
    matched_points2[i] = points2[idx2];
    matched_points1_normalized[i] = camera1.CamFromImg(points1[idx1]);
    matched_points2_normalized[i] = camera2.CamFromImg(points2[idx2]);
  }

  // Estimate epipolar models.

  auto E_ransac_options = options.ransac_options;
  E_ransac_options.max_error =
      (camera1.CamFromImgThreshold(options.ransac_options.max_error) +
       camera2.CamFromImgThreshold(options.ransac_options.max_error)) /
      2;

  LORANSAC<EssentialMatrixFivePointEstimator, EssentialMatrixFivePointEstimator>
      E_ransac(E_ransac_options);
  const auto E_report =
      E_ransac.Estimate(matched_points1_normalized, matched_points2_normalized);
  geometry.E = E_report.model;

  LORANSAC<FundamentalMatrixSevenPointEstimator,
           FundamentalMatrixEightPointEstimator>
      F_ransac(options.ransac_options);
  const auto F_report = F_ransac.Estimate(matched_points1, matched_points2);
  geometry.F = F_report.model;

  // Estimate planar or panoramic model.

  LORANSAC<HomographyMatrixEstimator, HomographyMatrixEstimator> H_ransac(
      options.ransac_options);
  const auto H_report = H_ransac.Estimate(matched_points1, matched_points2);
  geometry.H = H_report.model;

  if ((!E_report.success && !F_report.success && !H_report.success) ||
      (E_report.support.num_inliers < min_num_inliers &&
       F_report.support.num_inliers < min_num_inliers &&
       H_report.support.num_inliers < min_num_inliers)) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  // Determine inlier ratios of different models.

  const double E_F_inlier_ratio =
      static_cast<double>(E_report.support.num_inliers) /
      F_report.support.num_inliers;
  const double H_F_inlier_ratio =
      static_cast<double>(H_report.support.num_inliers) /
      F_report.support.num_inliers;
  const double H_E_inlier_ratio =
      static_cast<double>(H_report.support.num_inliers) /
      E_report.support.num_inliers;

  const std::vector<char>* best_inlier_mask = nullptr;
  size_t num_inliers = 0;

  if (E_report.success && E_F_inlier_ratio > options.min_E_F_inlier_ratio &&
      E_report.support.num_inliers >= min_num_inliers) {
    // Calibrated configuration.

    // Always use the model with maximum matches.
    if (E_report.support.num_inliers >= F_report.support.num_inliers) {
      num_inliers = E_report.support.num_inliers;
      best_inlier_mask = &E_report.inlier_mask;
    } else {
      num_inliers = F_report.support.num_inliers;
      best_inlier_mask = &F_report.inlier_mask;
    }

    if (H_E_inlier_ratio > options.max_H_inlier_ratio) {
      geometry.config = TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC;
      if (H_report.support.num_inliers > num_inliers) {
        num_inliers = H_report.support.num_inliers;
        best_inlier_mask = &H_report.inlier_mask;
      }
    } else {
      geometry.config = TwoViewGeometry::ConfigurationType::CALIBRATED;
    }
  } else if (F_report.success &&
             F_report.support.num_inliers >= min_num_inliers) {
    // Uncalibrated configuration.

    num_inliers = F_report.support.num_inliers;
    best_inlier_mask = &F_report.inlier_mask;

    if (H_F_inlier_ratio > options.max_H_inlier_ratio) {
      geometry.config = TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC;
      if (H_report.support.num_inliers > num_inliers) {
        num_inliers = H_report.support.num_inliers;
        best_inlier_mask = &H_report.inlier_mask;
      }
    } else {
      geometry.config = TwoViewGeometry::ConfigurationType::UNCALIBRATED;
    }
  } else if (H_report.success &&
             H_report.support.num_inliers >= min_num_inliers) {
    num_inliers = H_report.support.num_inliers;
    best_inlier_mask = &H_report.inlier_mask;
    geometry.config = TwoViewGeometry::ConfigurationType::PLANAR_OR_PANORAMIC;
  } else {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  if (best_inlier_mask != nullptr) {
    geometry.inlier_matches =
        ExtractInlierMatches(matches, num_inliers, *best_inlier_mask);

    if (options.detect_watermark && DetectWatermark(camera1,
                                                    matched_points1,
                                                    camera2,
                                                    matched_points2,
                                                    num_inliers,
                                                    *best_inlier_mask,
                                                    options)) {
      geometry.config = TwoViewGeometry::ConfigurationType::WATERMARK;
    }

    if (options.compute_relative_pose) {
      EstimateTwoViewGeometryPose(
          camera1, points1, camera2, points2, &geometry);
    }
  }

  return geometry;
}

bool DetectWatermark(const Camera& camera1,
                     const std::vector<Eigen::Vector2d>& points1,
                     const Camera& camera2,
                     const std::vector<Eigen::Vector2d>& points2,
                     const size_t num_inliers,
                     const std::vector<char>& inlier_mask,
                     const TwoViewGeometryOptions& options) {
  CHECK(options.Check());

  // Check if inlier points in border region and extract inlier matches.

  const double diagonal1 = std::sqrt(camera1.width * camera1.width +
                                     camera1.height * camera1.height);
  const double diagonal2 = std::sqrt(camera2.width * camera2.width +
                                     camera2.height * camera2.height);
  const double minx1 = options.watermark_border_size * diagonal1;
  const double miny1 = minx1;
  const double maxx1 = camera1.width - minx1;
  const double maxy1 = camera1.height - miny1;
  const double minx2 = options.watermark_border_size * diagonal2;
  const double miny2 = minx2;
  const double maxx2 = camera2.width - minx2;
  const double maxy2 = camera2.height - miny2;

  std::vector<Eigen::Vector2d> inlier_points1(num_inliers);
  std::vector<Eigen::Vector2d> inlier_points2(num_inliers);

  size_t num_matches_in_border = 0;

  size_t j = 0;
  for (size_t i = 0; i < inlier_mask.size(); ++i) {
    if (inlier_mask[i]) {
      const auto& point1 = points1[i];
      const auto& point2 = points2[i];

      inlier_points1[j] = point1;
      inlier_points2[j] = point2;
      j += 1;

      if (!IsImagePointInBoundingBox(point1, minx1, maxx1, miny1, maxy1) &&
          !IsImagePointInBoundingBox(point2, minx2, maxx2, miny2, maxy2)) {
        num_matches_in_border += 1;
      }
    }
  }

  const double matches_in_border_ratio =
      static_cast<double>(num_matches_in_border) / num_inliers;

  if (matches_in_border_ratio < options.watermark_min_inlier_ratio) {
    return false;
  }

  // Check if matches follow a translational model.

  RANSACOptions ransac_options = options.ransac_options;
  ransac_options.min_inlier_ratio = options.watermark_min_inlier_ratio;

  LORANSAC<TranslationTransformEstimator<2>, TranslationTransformEstimator<2>>
      ransac(ransac_options);
  const auto report = ransac.Estimate(inlier_points1, inlier_points2);

  const double inlier_ratio =
      static_cast<double>(report.support.num_inliers) / num_inliers;

  return inlier_ratio >= options.watermark_min_inlier_ratio;
}

TwoViewGeometry EstimateRefractiveTwoViewGeometry(
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Camera>& virtual_cameras1,
    const std::vector<Rigid3d>& virtual_from_reals1,
    const std::vector<Eigen::Vector2d>& points2,
    const std::vector<Camera>& virtual_cameras2,
    const std::vector<Rigid3d>& virtual_from_reals2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options,
    const bool refine) {
  TwoViewGeometry geometry;

  const size_t min_num_inliers = static_cast<size_t>(options.min_num_inliers);
  if (matches.size() < min_num_inliers) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  // Extract corresponding points.
  std::vector<GR6PEstimator::X_t> matched_points1(matches.size());
  std::vector<GR6PEstimator::Y_t> matched_points2(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    matched_points1[i].cam_from_rig =
        virtual_from_reals1[matches[i].point2D_idx1];
    matched_points1[i].ray_in_cam =
        virtual_cameras1[matches[i].point2D_idx1]
            .CamFromImg(points1[matches[i].point2D_idx1])
            .homogeneous()
            .normalized();

    matched_points2[i].cam_from_rig =
        virtual_from_reals2[matches[i].point2D_idx2];
    matched_points2[i].ray_in_cam =
        virtual_cameras2[matches[i].point2D_idx2]
            .CamFromImg(points2[matches[i].point2D_idx2])
            .homogeneous()
            .normalized();
  }

  RANSACOptions ransac_options_copy = options.ransac_options;
  // Give it more iterations for RANSAC.
  // ransac_options_copy.max_num_trials *= 10;
  ransac_options_copy.max_error =
      (virtual_cameras1[0].CamFromImgThreshold(ransac_options_copy.max_error) +
       virtual_cameras2[0].CamFromImgThreshold(ransac_options_copy.max_error)) /
      2;
  LORANSAC<GR6PEstimator, GR6PEstimator> ransac(ransac_options_copy);
  const auto report = ransac.Estimate(matched_points1, matched_points2);
  geometry.cam2_from_cam1 = report.model;

  if (!report.success || report.support.num_inliers < min_num_inliers) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  } else {
    geometry.config = TwoViewGeometry::ConfigurationType::REFRACTIVE;
  }

  // Generalized relative pose estimator outputs directly cam2_from_cam1.
  geometry.inlier_matches = ExtractInlierMatches(
      matches, report.support.num_inliers, report.inlier_mask);

  if (options.compute_relative_pose) {
    // Extract tri_angle after estimating the two-view geometry.
    std::vector<double> tri_angles;

    std::vector<Eigen::Vector2d> inlier_points1_normalized;
    std::vector<Eigen::Vector2d> inlier_points2_normalized;

    std::vector<Eigen::Matrix3x4d> inlier_virtual_proj_matrix1;
    std::vector<Eigen::Matrix3x4d> inlier_virtual_proj_matrix2;

    std::vector<Rigid3d> inlier_virtual_from_reals1;
    std::vector<Rigid3d> inlier_virtual_from_reals2;

    inlier_points1_normalized.reserve(geometry.inlier_matches.size());
    inlier_points2_normalized.reserve(geometry.inlier_matches.size());

    inlier_virtual_proj_matrix1.reserve(geometry.inlier_matches.size());
    inlier_virtual_proj_matrix2.reserve(geometry.inlier_matches.size());

    inlier_virtual_from_reals1.reserve(geometry.inlier_matches.size());
    inlier_virtual_from_reals2.reserve(geometry.inlier_matches.size());

    const Rigid3d real1_from_world;
    const Rigid3d real2_from_world = geometry.cam2_from_cam1;

    for (const auto& match : geometry.inlier_matches) {
      const Camera& virtual_camera1 = virtual_cameras1[match.point2D_idx1];
      const Camera& virtual_camera2 = virtual_cameras2[match.point2D_idx2];

      inlier_points1_normalized.push_back(
          virtual_camera1.CamFromImg(points1[match.point2D_idx1]));
      inlier_points2_normalized.push_back(
          virtual_camera2.CamFromImg(points2[match.point2D_idx2]));

      const Rigid3d virtual1_from_world =
          virtual_from_reals1[match.point2D_idx1] * real1_from_world;
      const Rigid3d virtual2_from_world =
          virtual_from_reals2[match.point2D_idx2] * real2_from_world;

      inlier_virtual_proj_matrix1.push_back(virtual1_from_world.ToMatrix());
      inlier_virtual_proj_matrix2.push_back(virtual2_from_world.ToMatrix());
      inlier_virtual_from_reals1.push_back(
          virtual_from_reals1[match.point2D_idx1]);
      inlier_virtual_from_reals2.push_back(
          virtual_from_reals2[match.point2D_idx2]);
    }

    if (refine) {
      if (!RefineRefractiveTwoViewGeometry(inlier_points1_normalized,
                                           inlier_virtual_from_reals1,
                                           inlier_points2_normalized,
                                           inlier_virtual_from_reals2,
                                           &geometry.cam2_from_cam1)) {
        // Optimization failed, directly return and clean up the inlier matches.
        geometry.inlier_matches.clear();
        geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
        return geometry;
      };
    }

    const double kMinDepth = std::numeric_limits<double>::epsilon();

    auto CalculateDepth = [](const Eigen::Matrix3x4d& cam_from_world,
                             const Eigen::Vector3d& point3D) {
      const double proj_z = cam_from_world.row(2).dot(point3D.homogeneous());
      return proj_z * cam_from_world.col(2).norm();
    };

    for (size_t i = 0; i < geometry.inlier_matches.size(); ++i) {
      const double max_depth =
          1000.0f *
          (inlier_virtual_proj_matrix2[i].block<3, 3>(0, 0).transpose() *
           inlier_virtual_proj_matrix2[i].col(3))
              .norm();
      const Eigen::Vector3d point3D =
          TriangulatePoint(inlier_virtual_proj_matrix1[i],
                           inlier_virtual_proj_matrix2[i],
                           inlier_points1_normalized[i],
                           inlier_points2_normalized[i]);

      const double depth1 =
          CalculateDepth(inlier_virtual_proj_matrix1[i], point3D);

      if (depth1 > kMinDepth && depth1 < max_depth) {
        const double depth2 =
            CalculateDepth(inlier_virtual_proj_matrix2[i], point3D);
        if (depth2 > kMinDepth && depth2 < max_depth) {
          const Eigen::Vector3d proj_center1 =
              -inlier_virtual_proj_matrix1[i].block<3, 3>(0, 0).transpose() *
              inlier_virtual_proj_matrix1[i].col(3);
          const Eigen::Vector3d proj_center2 =
              -inlier_virtual_proj_matrix2[i].block<3, 3>(0, 0).transpose() *
              inlier_virtual_proj_matrix2[i].col(3);
          tri_angles.push_back(
              CalculateTriangulationAngle(proj_center1, proj_center2, point3D));
        }
      }
    }

    if (tri_angles.size() < min_num_inliers) {
      geometry.tri_angle = 0;
      LOG(WARNING)
          << "Found sufficient amount of inliers, but `tri_angle` "
             "cannot be computed as points may not have positive depths"
          << std::endl;
    } else {
      geometry.tri_angle = Median(tri_angles);
    }
  }
  return geometry;
}

bool RefineRefractiveTwoViewGeometry(
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Rigid3d>& virtual_from_reals1,
    const std::vector<Eigen::Vector2d>& points2,
    const std::vector<Rigid3d>& virtual_from_reals2,
    Rigid3d* rig2_from_rig1) {
  // CostFunction assumes unit quaternions.
  rig2_from_rig1->rotation.normalize();

  double* rig2_from_rig1_rotation = rig2_from_rig1->rotation.coeffs().data();
  double* rig2_from_rig1_translation = rig2_from_rig1->translation.data();

  const double kMaxL2Error = 1.0;
  ceres::LossFunction* loss_function = new ceres::CauchyLoss(kMaxL2Error);

  ceres::Problem problem;
  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = 100;
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.minimizer_progress_to_stdout = false;

  // The overhead of creating threads is too large.
  solver_options.num_threads = 1;
#if CERES_VERSION_MAJOR < 2
  solver_options.num_linear_solver_threads = 1;
#endif  // CERES_VERSION_MAJOR

  for (size_t i = 0; i < points1.size(); ++i) {
    ceres::CostFunction* cost_function =
        GeneralizedSampsonErrorCostFunction::Create(points1[i],
                                                    points2[i],
                                                    virtual_from_reals1[i],
                                                    virtual_from_reals2[i]);
    problem.AddResidualBlock(cost_function,
                             loss_function,
                             rig2_from_rig1_rotation,
                             rig2_from_rig1_translation);
  }

  SetQuaternionManifold(&problem, rig2_from_rig1_rotation);

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  return summary.IsSolutionUsable();
}

Camera BestFitNonRefracCamera(const CameraModelId tgt_model_id,
                              const Camera& camera,
                              const double approx_depth) {
  CHECK(camera.IsCameraRefractive())
      << "Camera is not refractive, cannot compute the best approximated "
         "non-refractive camera";

  Camera tgt_camera = Camera::CreateFromModelId(camera.camera_id,
                                                tgt_model_id,
                                                camera.MeanFocalLength(),
                                                camera.width,
                                                camera.height);
  tgt_camera.SetPrincipalPointX(camera.PrincipalPointX());
  tgt_camera.SetPrincipalPointY(camera.PrincipalPointY());

  // Sample 2D-3D correspondences for optimization.
  const size_t kNumSamples = 1000;
  std::vector<Eigen::Vector2d> points2D(kNumSamples);
  std::vector<Eigen::Vector3d> points3D(kNumSamples);
  const double width = static_cast<double>(tgt_camera.width);
  const double height = static_cast<double>(tgt_camera.height);

  for (size_t i = 0; i < kNumSamples; ++i) {
    Eigen::Vector2d image_point(RandomUniformReal(0.5, width - 0.5),
                                RandomUniformReal(0.5, height - 0.5));
    Eigen::Vector3d world_point =
        camera.CamFromImgRefracPoint(image_point, approx_depth);
    points2D[i] = image_point;
    points3D[i] = world_point;
  }

  const Rigid3d identity_pose = Rigid3d();

  ceres::Problem problem;
  ceres::Solver::Summary summary;
  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = 100;
  solver_options.function_tolerance *= 1e-4;
  solver_options.gradient_tolerance *= 1e-4;

  double* camera_params = tgt_camera.params.data();

  for (size_t i = 0; i < kNumSamples; i++) {
    const Eigen::Vector2d& point2D = points2D[i];
    Eigen::Vector3d& point3D = points3D[i];

    ceres::CostFunction* cost_function = nullptr;
    switch (tgt_camera.model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                        \
  case CameraModel::model_id:                                                 \
    cost_function = ReprojErrorConstantPoseCostFunction<CameraModel>::Create( \
        identity_pose, point2D);                                              \
    break;

      CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
    }

    problem.AddResidualBlock(
        cost_function, nullptr, point3D.data(), camera_params);

    problem.SetParameterBlockConstant(point3D.data());
  }

  solver_options.minimizer_progress_to_stdout = false;
  ceres::Solve(solver_options, &problem, &summary);

  // if optimization failed, use the original parameters
  if (!summary.IsSolutionUsable() ||
      summary.termination_type == ceres::TerminationType::NO_CONVERGENCE) {
    LOG(WARNING) << "Failed to compute the best fit persepctive model, taking "
                    "the original intrinsic parameters";
    tgt_camera.model_id = camera.model_id;
    tgt_camera.params = camera.params;
  } else {
    // Check for residuals.
    double reproj_error_sum = 0.0;
    for (size_t i = 0; i < kNumSamples; i++) {
      const double squared_reproj_error = CalculateSquaredReprojectionError(
          points2D[i], points3D[i], identity_pose, tgt_camera, false);
      reproj_error_sum += std::sqrt(squared_reproj_error);
    }
    reproj_error_sum = reproj_error_sum / static_cast<double>(kNumSamples);
    LOG(INFO) << "Best fit parameters for model " << tgt_camera.ModelName()
              << " in distance " << approx_depth
              << " computed, average residual: " << reproj_error_sum
              << std::endl;
  }
  return tgt_camera;
}

TwoViewGeometry EstimateRefractiveTwoViewGeometryUseBestFit(
    const Camera& best_fit_camera1,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Camera>& virtual_cameras1,
    const std::vector<Rigid3d>& virtual_from_reals1,
    const Camera& best_fit_camera2,
    const std::vector<Eigen::Vector2d>& points2,
    const std::vector<Camera>& virtual_cameras2,
    const std::vector<Rigid3d>& virtual_from_reals2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options,
    const bool refine) {
  // Perform calibrated two-view geometry estimation just like non-refractive
  // case.
  TwoViewGeometry geometry = EstimateCalibratedTwoViewGeometry(
      best_fit_camera1, points1, best_fit_camera2, points2, matches, options);

  if (geometry.config != TwoViewGeometry::ConfigurationType::DEGENERATE) {
    geometry.config = TwoViewGeometry::ConfigurationType::REFRACTIVE;
  }

  if (refine) {
    // Perform refinement afterwards:
    std::vector<Eigen::Vector2d> inlier_points1_normalized;
    std::vector<Eigen::Vector2d> inlier_points2_normalized;

    std::vector<Rigid3d> inlier_virtual_from_reals1;
    std::vector<Rigid3d> inlier_virtual_from_reals2;

    inlier_points1_normalized.reserve(geometry.inlier_matches.size());
    inlier_points2_normalized.reserve(geometry.inlier_matches.size());

    inlier_virtual_from_reals1.reserve(geometry.inlier_matches.size());
    inlier_virtual_from_reals2.reserve(geometry.inlier_matches.size());

    for (const auto& match : geometry.inlier_matches) {
      const Camera& virtual_camera1 = virtual_cameras1[match.point2D_idx1];
      const Camera& virtual_camera2 = virtual_cameras2[match.point2D_idx2];

      inlier_points1_normalized.push_back(
          virtual_camera1.CamFromImg(points1[match.point2D_idx1]));
      inlier_points2_normalized.push_back(
          virtual_camera2.CamFromImg(points2[match.point2D_idx2]));

      inlier_virtual_from_reals1.push_back(
          virtual_from_reals1[match.point2D_idx1]);
      inlier_virtual_from_reals2.push_back(
          virtual_from_reals2[match.point2D_idx2]);
    }

    if (!RefineRefractiveTwoViewGeometry(inlier_points1_normalized,
                                         inlier_virtual_from_reals1,
                                         inlier_points2_normalized,
                                         inlier_virtual_from_reals2,
                                         &geometry.cam2_from_cam1)) {
      // Optimization failed, directly return and clean up the inlier matches.
      geometry.inlier_matches.clear();
      geometry.config = TwoViewGeometry::DEGENERATE;
      return geometry;
    };
  }
  return geometry;
}

TwoViewGeometry EstimateRefractiveTwoViewGeometryHu(
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Camera>& virtual_cameras1,
    const std::vector<Rigid3d>& virtual_from_reals1,
    const std::vector<Eigen::Vector2d>& points2,
    const std::vector<Camera>& virtual_cameras2,
    const std::vector<Rigid3d>& virtual_from_reals2,
    const FeatureMatches& matches,
    const TwoViewGeometryOptions& options,
    const bool refine) {
  TwoViewGeometry geometry;

  const size_t min_num_inliers = static_cast<size_t>(options.min_num_inliers);
  if (matches.size() < min_num_inliers) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  }

  // Extract corresponding points.
  std::vector<RefracRelPoseEstimator::X_t> matched_points1(matches.size());
  std::vector<RefracRelPoseEstimator::Y_t> matched_points2(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    matched_points1[i].virtual_from_real =
        virtual_from_reals1[matches[i].point2D_idx1];
    matched_points1[i].ray_in_virtual =
        virtual_cameras1[matches[i].point2D_idx1]
            .CamFromImg(points1[matches[i].point2D_idx1])
            .homogeneous()
            .normalized();

    matched_points2[i].virtual_from_real =
        virtual_from_reals2[matches[i].point2D_idx2];
    matched_points2[i].ray_in_virtual =
        virtual_cameras2[matches[i].point2D_idx2]
            .CamFromImg(points2[matches[i].point2D_idx2])
            .homogeneous()
            .normalized();
  }

  RANSACOptions ransac_options_copy = options.ransac_options;
  // Give it more iterations for RANSAC.
  // ransac_options_copy.max_num_trials *= 10;
  ransac_options_copy.max_error =
      (virtual_cameras1[0].CamFromImgThreshold(ransac_options_copy.max_error) +
       virtual_cameras2[0].CamFromImgThreshold(ransac_options_copy.max_error)) /
      2;
  LORANSAC<RefracRelPoseEstimator, RefracRelPoseEstimator> ransac(
      ransac_options_copy);
  const auto report = ransac.Estimate(matched_points1, matched_points2);
  geometry.cam2_from_cam1 = report.model;

  if (!report.success || report.support.num_inliers < min_num_inliers) {
    geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
    return geometry;
  } else {
    geometry.config = TwoViewGeometry::ConfigurationType::REFRACTIVE;
  }

  // Generalized relative pose estimator outputs directly cam2_from_cam1.
  geometry.inlier_matches = ExtractInlierMatches(
      matches, report.support.num_inliers, report.inlier_mask);

  if (options.compute_relative_pose) {
    // Extract tri_angle after estimating the two-view geometry.
    std::vector<double> tri_angles;

    std::vector<Eigen::Vector2d> inlier_points1_normalized;
    std::vector<Eigen::Vector2d> inlier_points2_normalized;

    std::vector<Eigen::Matrix3x4d> inlier_virtual_proj_matrix1;
    std::vector<Eigen::Matrix3x4d> inlier_virtual_proj_matrix2;

    std::vector<Rigid3d> inlier_virtual_from_reals1;
    std::vector<Rigid3d> inlier_virtual_from_reals2;

    inlier_points1_normalized.reserve(geometry.inlier_matches.size());
    inlier_points2_normalized.reserve(geometry.inlier_matches.size());

    inlier_virtual_proj_matrix1.reserve(geometry.inlier_matches.size());
    inlier_virtual_proj_matrix2.reserve(geometry.inlier_matches.size());

    inlier_virtual_from_reals1.reserve(geometry.inlier_matches.size());
    inlier_virtual_from_reals2.reserve(geometry.inlier_matches.size());

    const Rigid3d real1_from_world;
    const Rigid3d real2_from_world = geometry.cam2_from_cam1;

    for (const auto& match : geometry.inlier_matches) {
      const Camera& virtual_camera1 = virtual_cameras1[match.point2D_idx1];
      const Camera& virtual_camera2 = virtual_cameras2[match.point2D_idx2];

      inlier_points1_normalized.push_back(
          virtual_camera1.CamFromImg(points1[match.point2D_idx1]));
      inlier_points2_normalized.push_back(
          virtual_camera2.CamFromImg(points2[match.point2D_idx2]));

      const Rigid3d virtual1_from_world =
          virtual_from_reals1[match.point2D_idx1] * real1_from_world;
      const Rigid3d virtual2_from_world =
          virtual_from_reals2[match.point2D_idx2] * real2_from_world;

      inlier_virtual_proj_matrix1.push_back(virtual1_from_world.ToMatrix());
      inlier_virtual_proj_matrix2.push_back(virtual2_from_world.ToMatrix());
      inlier_virtual_from_reals1.push_back(
          virtual_from_reals1[match.point2D_idx1]);
      inlier_virtual_from_reals2.push_back(
          virtual_from_reals2[match.point2D_idx2]);
    }

    if (refine) {
      if (!RefineRefractiveTwoViewGeometry(inlier_points1_normalized,
                                           inlier_virtual_from_reals1,
                                           inlier_points2_normalized,
                                           inlier_virtual_from_reals2,
                                           &geometry.cam2_from_cam1)) {
        // Optimization failed, directly return and clean up the inlier matches.
        geometry.inlier_matches.clear();
        geometry.config = TwoViewGeometry::ConfigurationType::DEGENERATE;
        return geometry;
      };
    }

    const double kMinDepth = std::numeric_limits<double>::epsilon();

    auto CalculateDepth = [](const Eigen::Matrix3x4d& cam_from_world,
                             const Eigen::Vector3d& point3D) {
      const double proj_z = cam_from_world.row(2).dot(point3D.homogeneous());
      return proj_z * cam_from_world.col(2).norm();
    };

    for (size_t i = 0; i < geometry.inlier_matches.size(); ++i) {
      const double max_depth =
          1000.0f *
          (inlier_virtual_proj_matrix2[i].block<3, 3>(0, 0).transpose() *
           inlier_virtual_proj_matrix2[i].col(3))
              .norm();
      const Eigen::Vector3d point3D =
          TriangulatePoint(inlier_virtual_proj_matrix1[i],
                           inlier_virtual_proj_matrix2[i],
                           inlier_points1_normalized[i],
                           inlier_points2_normalized[i]);

      const double depth1 =
          CalculateDepth(inlier_virtual_proj_matrix1[i], point3D);

      if (depth1 > kMinDepth && depth1 < max_depth) {
        const double depth2 =
            CalculateDepth(inlier_virtual_proj_matrix2[i], point3D);
        if (depth2 > kMinDepth && depth2 < max_depth) {
          const Eigen::Vector3d proj_center1 =
              -inlier_virtual_proj_matrix1[i].block<3, 3>(0, 0).transpose() *
              inlier_virtual_proj_matrix1[i].col(3);
          const Eigen::Vector3d proj_center2 =
              -inlier_virtual_proj_matrix2[i].block<3, 3>(0, 0).transpose() *
              inlier_virtual_proj_matrix2[i].col(3);
          tri_angles.push_back(
              CalculateTriangulationAngle(proj_center1, proj_center2, point3D));
        }
      }
    }

    if (tri_angles.size() < min_num_inliers) {
      geometry.tri_angle = 0;
      LOG(WARNING)
          << "Found sufficient amount of inliers, but `tri_angle` "
             "cannot be computed as points may not have positive depths"
          << std::endl;
    } else {
      geometry.tri_angle = Median(tri_angles);
    }
  }
  return geometry;
}

}  // namespace colmap
