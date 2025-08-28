#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "calibmar/extractors/aruco_board_extractor.h"
#include "utils/memory_image_reader.h"
#include "utils/utils.h"

#include "calibmar/readers/filesystem_reader.h"
#include <Eigen/Core>
#include <fstream>
#include <iostream>

using namespace calibmar;

namespace {
  extern std::vector<Eigen::Vector2d> expected_aruco_grid_points_2D;
  extern std::vector<Eigen::Vector2d> expected_april_grid_points_2D;
  extern std::vector<std::vector<Eigen::Vector3d>> points_3D;
}

// regression test, actual values not known
BOOST_TEST_DONT_PRINT_LOG_VALUE(FeatureExtractor::Status)
BOOST_AUTO_TEST_CASE(BasicArucoBoardExtraction) {
  ArucoBoardFeatureExtractor::Options options;
  options.marker_cols = 5;
  options.marker_rows = 7;
  options.marker_size = 10;
  options.marker_spacing = 10;
  options.aruco_type = ArucoMarkerTypes::DICT_6X6;
  MemoryImageReader reader({MemoryImageReader::Images::ArucoGridBoard});
  ArucoBoardFeatureExtractor extractor(options);

  Image image;
  Pixmap pixmap;
  reader.Next(image, pixmap);
  FeatureExtractor::Status status = extractor.Extract(image, pixmap);

  BOOST_TEST(status == FeatureExtractor::Status::SUCCESS);

  const std::vector<Eigen::Vector2d>& points_2D = image.Points2D();
  for (size_t i = 0; i < points_2D.size(); i++) {
    const Eigen::Vector2d point = points_2D[i];
    Eigen::Vector2d point_expected = expected_aruco_grid_points_2D[i];

    BOOST_TEST(point.isApprox(point_expected, 0.05));
  }
}

// regression test, actual values not known
BOOST_AUTO_TEST_CASE(AprilTagArucoBoardExtraction) {
  ArucoBoardFeatureExtractor::Options options;
  options.marker_cols = 6;
  options.marker_rows = 6;
  options.marker_size = 1;
  options.marker_spacing = 0.05;
  options.border_bits = 2;
  options.aruco_type = ArucoMarkerTypes::DICT_APRILTAG_36h11;

  MemoryImageReader reader({MemoryImageReader::Images::AprilGridBoard});
  ArucoBoardFeatureExtractor extractor(options);

  Image image;
  Pixmap pixmap;
  reader.Next(image, pixmap);
  FeatureExtractor::Status status = extractor.Extract(image, pixmap);

  BOOST_TEST(status == FeatureExtractor::Status::SUCCESS);

  const std::vector<Eigen::Vector2d>& points_2D = image.Points2D();
  for (size_t i = 0; i < points_2D.size(); i++) {
    const Eigen::Vector2d point = points_2D[i];
    Eigen::Vector2d point_expected = expected_april_grid_points_2D[i];

    BOOST_TEST(point.isApprox(point_expected, 0.05));
  }
}

struct ArucoOrderingTestData {
  ArucoGridOrigin origin;
  ArucoGridDirection direction;
  std::vector<size_t> id_indices;
};

BOOST_TEST_DONT_PRINT_LOG_VALUE(ArucoOrderingTestData)
std::vector<ArucoOrderingTestData> aruco_data = {
    {ArucoGridOrigin::TopLeft, ArucoGridDirection::Horizontal, {0, 1, 2, 3, 4, 5}},
    {ArucoGridOrigin::TopRight, ArucoGridDirection::Horizontal, {2, 1, 0, 5, 4, 3}},
    {ArucoGridOrigin::BottomLeft, ArucoGridDirection::Horizontal, {3, 4, 5, 0, 1, 2}},
    {ArucoGridOrigin::BottomRight, ArucoGridDirection::Horizontal, {5, 4, 3, 2, 1, 0}},
    {ArucoGridOrigin::TopLeft, ArucoGridDirection::Vertical, {0, 3, 1, 4, 2, 5}},
    {ArucoGridOrigin::TopRight, ArucoGridDirection::Vertical, {2, 5, 1, 4, 0, 3}},
    {ArucoGridOrigin::BottomLeft, ArucoGridDirection::Vertical, {3, 0, 4, 1, 5, 2}},
    {ArucoGridOrigin::BottomRight, ArucoGridDirection::Vertical, {5, 2, 4, 1, 3, 0}}};
BOOST_DATA_TEST_CASE(Ordering3DPoints, boost::unit_test::data::make(aruco_data), data) {
  ArucoBoardFeatureExtractor::Options options;
  options.marker_cols = 3;
  options.marker_rows = 2;
  options.marker_size = 1;
  options.marker_spacing = 1;
  options.grid_origin = data.origin;
  options.grid_direction = data.direction;

  ArucoBoardFeatureExtractor extractor(options);

  std::vector<Eigen::Vector3d> pts;
  for (size_t i = 0; i < points_3D.size(); i++) {
    for (size_t j = 0; j < points_3D[i].size(); j++) {
      pts.push_back(points_3D[data.id_indices[i]][j]);
    }
  }

  size_t i = 0;
  for (const auto& id_point : extractor.Points3D()) {
    BOOST_TEST(id_point.second == pts[i]);
    i++;
  }
}

namespace {
  std::vector<Eigen::Vector2d> expected_aruco_grid_points_2D{
      {373.369, 343.623}, {308.255, 341.072}, {307.335, 280.204}, {369.763, 283.198}, {301.087, 340.745}, {235.749, 337.997},
      {238.257, 277.719}, {300.957, 280.082}, {228.582, 337.649}, {162.792, 336.216}, {168.436, 275.733}, {231.154, 277.285},
      {155.417, 335.891}, {89.643, 333.629},  {98.7046, 273.695}, {161.278, 275.119}, {82.1074, 333.397}, {15.816, 330.791},
      {28.3896, 270.8},   {91.7895, 273.35},  {369.188, 276.113}, {307.353, 273.595}, {306.068, 220.732}, {365.39, 223.287},
      {301.027, 273.303}, {238.712, 270.795}, {240.368, 217.713}, {299.628, 220.215}, {231.519, 270.73},  {169.072, 268.643},
      {174.311, 215.769}, {233.908, 217.92},  {162.034, 268.215}, {99.7214, 266.796}, {108.043, 213.604}, {167.434, 215.382},
      {92.8461, 266.83},  {29.9021, 263.986}, {41.2015, 210.728}, {101.217, 213.112}, {364.944, 217.247}, {306.282, 214.711},
      {304.763, 165.854}, {361.381, 168.396}, {300.342, 214.299}, {240.781, 211.884}, {242.743, 163.773}, {298.931, 166.044},
      {234.228, 212.036}, {175.029, 209.799}, {179.475, 160.89},  {236.278, 163.44},  {168.338, 209.273}, {108.694, 207.788},
      {116.225, 158.001}, {173.223, 160.519}, {102.117, 207.11},  {42.5278, 204.805}, {52.5692, 155.642}, {109.839, 158.067},
      {361.159, 162.821}, {305.158, 160.419}, {304.737, 115.847}, {358.391, 118.211}, {299.03, 160.198},  {242.939, 158.33},
      {244.791, 112.77},  {298.939, 115.442}, {236.584, 158.142}, {180.09, 155.029},  {184.362, 109.767}, {238.772, 112.375},
      {173.553, 154.83},  {116.935, 152.545}, {123.807, 108.07},  {178.255, 109.853}, {110.536, 152.404}, {53.7052, 150.038},
      {62.9038, 104.997}, {117.942, 107.953}, {358.089, 113.361}, {303.955, 110.178}, {304.092, 69.3611}, {355.48, 72.1461},
      {299.043, 110.269}, {244.986, 107.545}, {246.596, 66.8038}, {298.911, 69.2719}, {238.828, 107.237}, {184.83, 104.693},
      {188.821, 64.786},  {240.932, 66.3159}, {178.627, 104.693}, {124.69, 103.011},  {130.861, 62.0912}, {182.987, 64.3342},
      {118.565, 102.927}, {63.9825, 99.8857}, {72.5388, 58.884},  {125.07, 62.0301},  {355.231, 67.2067}, {303.95, 64.6453},
      {302.737, 27.0439}, {352.803, 29.0473}, {298.147, 64.221},  {246.803, 62.001},  {248.284, 24.7124}, {297.919, 27.0736},
      {240.841, 61.7251}, {189.335, 59.7509}, {192.877, 21.9004}, {242.862, 24.4517}, {183.513, 59.4366}, {131.68, 57.4836},
      {137.263, 19.6242}, {187.258, 21.9861}, {125.867, 57.2821}, {73.6353, 54.0973}, {81.56, 17.0912},   {131.72, 19.2638}};

  std::vector<Eigen::Vector2d> expected_april_grid_points_2D{
      {111.318, 60.9961}, {108.016, 99.9721}, {58.7893, 99.9698}, {63.4921, 61.0367}, {107.16, 112.07},   {103.691, 156.242},
      {52.1609, 155.347}, {57.363, 111.98},   {102.665, 169.013}, {99.0637, 215.949}, {44.7877, 216.013}, {50.5977, 169},
      {97.7696, 231.795}, {93.5146, 284.008}, {36.1699, 283.984}, {42.6463, 232.126}, {92.1294, 300.707}, {87.2594, 359.807},
      {26.9086, 360.346}, {33.9454, 301.993}, {173.301, 60.9885}, {171.885, 100.027}, {123.259, 99.9471}, {125.908, 61.0232},
      {172.227, 113},     {170.743, 156},     {119.782, 156},     {122.279, 113},     {170.477, 169.009}, {169.757, 215.981},
      {115.714, 216.009}, {118.862, 169.012}, {169.225, 230.995}, {167.717, 284.034}, {110.995, 283.929}, {114.538, 231.009},
      {167.419, 301},     {165.894, 359},     {105.779, 359},     {109.917, 301},     {233.06, 14},       {234.327, 50},
      {187.987, 50},      {188.026, 14},      {234.652, 60.9898}, {235.9, 99.9832},   {186.73, 100.009},  {187.727, 61.0208},
      {236.125, 113},     {237.337, 156.012}, {186.985, 155.975}, {187.029, 113},     {237.843, 168.99},  {239.31, 216},
      {185.766, 216},     {186.659, 169.021}, {239.902, 230.99},  {241.392, 282.828}, {185.321, 284.282}, {186.263, 231.02},
      {242.031, 299.865}, {243.958, 359.01},  {184.883, 358.96},  {185.066, 301.3},   {291.48, 15.2922},  {294.273, 49.9399},
      {248.272, 50.0303}, {246.831, 14.2151}, {295.498, 62.2789}, {298.635, 100.881}, {250.274, 99.7153}, {248.861, 60.8126},
      {299.654, 113},     {303.285, 156},     {252.999, 156},     {250.86, 113},      {304.415, 168.948}, {308.366, 215.974},
      {255.372, 215.998}, {253.404, 169.104}, {309.775, 230.971}, {314.298, 283.006}, {258.453, 282.962}, {256.122, 231.099},
      {315.713, 299.95},  {320.974, 357.945}, {261.959, 358.087}, {259.145, 300.096}, {349.869, 15.3231}, {354.482, 50.4362},
      {308.345, 49.9182}, {305.151, 16.2593}, {356.232, 62.0734}, {361.829, 100.903}, {313.226, 99.7106}, {309.754, 61.8369},
      {363.367, 113},     {369.319, 154.931}, {318.563, 155.115}, {314.676, 113},     {371.306, 168.991}, {377.866, 215.971},
      {324.499, 215.97},  {319.909, 169.046}, {379.959, 230.992}, {387.303, 282.985}, {331.222, 283.007}, {325.891, 231.048},
      {389.787, 299.991}, {397.888, 357.992}, {338.933, 357.987}, {332.956, 300.051}, {427.409, 112.049}, {435.382, 154.998},
      {385.219, 154.984}, {378.671, 113.28},  {438.004, 168.424}, {447.449, 214.736}, {394.301, 215.567}, {387.496, 169.234},
      {463.656, 299.315}, {473.871, 356.703}, {416.091, 357.814}, {407.24, 300.266}};

  std::vector<std::vector<Eigen::Vector3d>> points_3D{
      {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}, {{2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0}},
      {{4, 0, 0}, {5, 0, 0}, {5, 1, 0}, {4, 1, 0}}, {{0, 2, 0}, {1, 2, 0}, {1, 3, 0}, {0, 3, 0}},
      {{2, 2, 0}, {3, 2, 0}, {3, 3, 0}, {2, 3, 0}}, {{4, 2, 0}, {5, 2, 0}, {5, 3, 0}, {4, 3, 0}}};
}