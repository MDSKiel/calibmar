#pragma once

#include "colmap/geometry/rigid3.h"

namespace colmap {

// Simple class to read pose prior from csv files
class PosePrior {
 public:
  PosePrior();

  bool Read(const std::string& path,
            Rigid3d* prior_from_world,
            Eigen::Matrix7d* prior_from_world_cov);

 private:
  double lat0_;
  double lon0_;
  double depth0_;
};

}  // namespace colmap