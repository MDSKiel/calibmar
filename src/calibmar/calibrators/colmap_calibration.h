#pragma once

#include "calibmar/core/calibration.h"

#include "colmap/controllers/incremental_mapper.h"
#include <colmap/estimators/two_view_geometry.h>
#include <colmap/feature/sift.h>
#include <colmap/scene/reconstruction.h>

namespace colmap_calibration {

  void CalibrateCamera(calibmar::Calibration& calibration, std::shared_ptr<colmap::Reconstruction>& reconstruction,
                       bool enable_refraction);

  void CalibrateCamera(calibmar::Calibration& calibration, std::shared_ptr<colmap::Reconstruction>& reconstruction,
                       bool enable_refraction, colmap::SiftMatchingOptions& matching_options,
                       colmap::TwoViewGeometryOptions& geometry_options, colmap::IncrementalMapperOptions& mapper_options);

}