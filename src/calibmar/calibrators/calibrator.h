#pragma once

#include "calibmar/core/calibration.h"

namespace calibmar {
  class Calibrator {
   public:
    virtual ~Calibrator() = default;

    virtual void Calibrate(Calibration& calibration) = 0;
  };
}