#pragma once

#include "calibmar/core/calibration.h"
#include "calibmar/readers/image_reader.h"
#include "extractor.h"
#include <colmap/feature/extractor.h>

namespace calibmar {

  class SiftFeatureExtractor : public FeatureExtractor {
   public:
    struct Options {
    };

    SiftFeatureExtractor(const Options& options);

    Status Extract(Image& image, const Pixmap& pixmap) override;

   private:
    Options options_;
    std::unique_ptr<colmap::FeatureExtractor> sift_extractor_;
  };
}