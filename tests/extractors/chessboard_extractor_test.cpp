#include "calibmar/extractors/chessboard_extractor.h"
#include "utils/memory_image_reader.h"

#include <boost/test/unit_test.hpp>

#include "calibmar/readers/filesystem_reader.h"
#include <Eigen/Core>
#include <fstream>
#include <iostream>

using namespace calibmar;

namespace {
  extern std::vector<Eigen::Vector2d> points_2D_expected;
}

// regression test, actual values not known
BOOST_AUTO_TEST_CASE(BasicChessBoardExtraction) {
  ChessboardFeatureExtractor::Options options;
  options.chessboard_columns = 10;
  options.chessboard_rows = 7;
  MemoryImageReader reader({MemoryImageReader::Images::BasicChessboard});
  ChessboardFeatureExtractor extractor(options);

  Image image;
  Pixmap pixmap;
  reader.Next(image, pixmap);
  FeatureExtractor::Status status = extractor.Extract(image, pixmap);

  const std::vector<Eigen::Vector2d>& points_2D = image.Points2D();
  for (size_t i = 0; i < points_2D.size(); i++) {
    const Eigen::Vector2d point = points_2D[i];
    Eigen::Vector2d point_expected = points_2D_expected[i];

    BOOST_TEST(point.isApprox(point_expected, 0.05));
  }
}

namespace {
  std::vector<Eigen::Vector2d> points_2D_expected{
      {154.412, 91.7825}, {136.927, 91.4767}, {119.753, 91.0201}, {102.754, 90.6487}, {86.303, 90.4194},  {69.7359, 90.0947},
      {53.5296, 89.69},   {37.5335, 89.555},  {21.5506, 89.4917}, {152.571, 75.3268}, {135.549, 74.9879}, {118.966, 74.6795},
      {102.444, 74.492},  {86.2847, 74.4187}, {70.3507, 74.2228}, {54.4462, 73.8685}, {38.7147, 73.8166}, {23.2516, 73.6841},
      {151.157, 59.4599}, {134.456, 59.4193}, {118.233, 59.3852}, {102.052, 59.295},  {86.2821, 59.0238}, {70.5727, 58.7978},
      {55.2328, 58.7154}, {39.6629, 58.7225}, {24.5089, 58.6353}, {149.561, 44.3538}, {133.405, 44.3809}, {117.458, 44.3999},
      {101.607, 44.4166}, {86.2807, 44.3839}, {70.8798, 44.3619}, {55.7146, 44.3827}, {40.7397, 44.3657}, {25.7761, 44.3723},
      {148.288, 29.8068}, {132.433, 29.8078}, {116.686, 30.0667}, {101.39, 30.2945},  {86.2753, 30.3827}, {71.3571, 30.4327},
      {56.4705, 30.4541}, {41.666, 30.4927},  {27.1397, 30.4448}, {146.937, 16.0139}, {131.47, 16.0312},  {116.287, 16.2738},
      {101.185, 16.5285}, {86.3006, 16.7238}, {71.5301, 17.088},  {57.1154, 17.1123}, {42.586, 17.254},   {28.3339, 17.3089}};
}