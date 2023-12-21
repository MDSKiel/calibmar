#include "calibmar/calibrators/basic_calibrator.h"

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "utils/test_helpers.h"

#include <iomanip>

using namespace calibmar;

namespace {
  double epsilon = 0.0001;
  std::pair<int, int> image_size{270, 180};
  void PrepareCalibration(Calibration& calibration);
  std::vector<double> GetExpectedParams(CameraModelType model);
}

BOOST_AUTO_TEST_CASE(BasicCalibration) {
  Calibration calibration;
  PrepareCalibration(calibration);
  BasicCalibrator::Options options;
  options.image_size = image_size;
  BasicCalibrator calibrator(options);

  calibrator.Calibrate(calibration);

  std::vector<double> params = calibration.Camera().Params();
  std::vector<double> expected_params = GetExpectedParams(CameraModelType::OpenCVCameraModel);
  Eigen::Map<Eigen::VectorXd> actual(params.data(), params.size());
  Eigen::Map<Eigen::VectorXd> expected(expected_params.data(), expected_params.size());
  BOOST_TEST(ElementWiseClose(actual, expected, epsilon));
}

BOOST_TEST_DONT_PRINT_LOG_VALUE(CameraModelType)
std::vector<CameraModelType> models = {CameraModelType::SimplePinholeCameraModel, CameraModelType::PinholeCameraModel,
                                       CameraModelType::SimpleRadialCameraModel,  CameraModelType::RadialCameraModel,
                                       CameraModelType::OpenCVCameraModel,        CameraModelType::FullOpenCVCameraModel,
                                       CameraModelType::OpenCVFisheyeCameraModel};
// This is more of a regression test. The expected values are not necessarily correct, but used to detect changes.
BOOST_DATA_TEST_CASE(CalibrationModels, boost::unit_test::data::make(models), model) {
  Calibration calibration;
  BasicCalibrator::Options options;
  options.camera_model = model;
  options.image_size = image_size;
  PrepareCalibration(calibration);
  BasicCalibrator calibrator(options);

  calibrator.Calibrate(calibration);

  std::vector<double> params = calibration.Camera().Params();
  std::vector<double> expected_params = GetExpectedParams(model);
  Eigen::Map<Eigen::VectorXd> actual(params.data(), params.size());
  Eigen::Map<Eigen::VectorXd> expected(expected_params.data(), expected_params.size());
  BOOST_TEST(ElementWiseClose(actual, expected, epsilon));
}

std::vector<CameraModelType> std_dev_models = {CameraModelType::SimplePinholeCameraModel, CameraModelType::PinholeCameraModel,
                                               CameraModelType::SimpleRadialCameraModel,  CameraModelType::RadialCameraModel,
                                               CameraModelType::OpenCVCameraModel,        CameraModelType::FullOpenCVCameraModel};
// OpenCV fisheye calibration does not support returning std deviations
BOOST_DATA_TEST_CASE(IntrinsicsDeviationMatchesParamsLength, boost::unit_test::data::make(std_dev_models), model) {
  Calibration calibration;
  BasicCalibrator::Options options;
  options.camera_model = model;
  options.image_size = image_size;
  PrepareCalibration(calibration);
  BasicCalibrator calibrator(options);

  calibrator.Calibrate(calibration);

  std::vector<double> params = calibration.Camera().Params();
  std::vector<double> intrinsics_std_dev = calibration.IntrinsicsStdDeviations();
  if (model == CameraModelType::PinholeCameraModel || model == CameraModelType::SimplePinholeCameraModel) {
    // For pinhole models the principal point is not estimated and therefore does not have a std deviation
    BOOST_TEST(params.size() - 2 == intrinsics_std_dev.size());
  }
  else {
    BOOST_TEST(params.size() == intrinsics_std_dev.size());
  }
}

// The expected values are not necessarily correct, but used to detect changes.
BOOST_AUTO_TEST_CASE(IntrinsicGuess) {
  Calibration calibration;
  PrepareCalibration(calibration);
  calibration.SetCamera(CameraModel::InitCamera(CameraModelType::PinholeCameraModel, image_size, {300, 300, 135, 90}));
  BasicCalibrator::Options options;
  options.use_intrinsics_guess = true;
  BasicCalibrator calibrator(options);

  calibrator.Calibrate(calibration);

  std::vector<double> params = calibration.Camera().Params();
  std::vector<double> expected_params = GetExpectedParams(CameraModelType::PinholeCameraModel);
  Eigen::Map<Eigen::VectorXd> actual(params.data(), params.size());
  Eigen::Map<Eigen::VectorXd> expected(expected_params.data(), expected_params.size());
  BOOST_TEST(ElementWiseClose(actual, expected, epsilon));
}

namespace {
  extern std::vector<Eigen::Vector3d> points3D;
  extern std::vector<std::vector<Eigen::Vector2d>> point2DSets;

  void PrepareCalibration(Calibration& calibration) {
    for (auto& point3D : points3D) {
      calibration.AddPoint3D(point3D);
    }

    for (auto& set : point2DSets) {
      Image image;

      image.SetPoints2D(set);

      for (size_t i = 0; i < set.size(); i++) {
        image.SetPoint3DforPoint2D(i, i);
      }

      calibration.AddImage(image);
    }
  }

  std::vector<double> GetExpectedParams(CameraModelType model) {
    switch (model) {
      case calibmar::CameraModelType::SimplePinholeCameraModel:
        return {225.68526725553883, 135, 90};
      case calibmar::CameraModelType::PinholeCameraModel:
        return {196.30181001486221, 217.95446625739132, 135, 90};
      case calibmar::CameraModelType::SimpleRadialCameraModel:
        return {215.07699224946091, 124.06171118648342, 71.408220362731569, -0.15052033210101046};
      case calibmar::CameraModelType::RadialCameraModel:
        return {214.68558101270199, 64.299960564912411, 67.100565326447395, -0.28463262584988541, 0.095363651359414839};
      case calibmar::CameraModelType::OpenCVCameraModel:
        return {2.0188715078245809e+02,  2.1747919060681681e+02, 9.6243850990938085e+01,  7.2696251399402101e+01,
                -1.4415600464693909e-01, 8.7045274598903946e-02, -1.2333462790628771e-02, -2.8774769110121036e-02};
      case calibmar::CameraModelType::FullOpenCVCameraModel:
        return {193.232371,     210.251722,    109.709759, 91.0455827, -18.976556, 94.9240243,
                -0.00211222641, -0.0329466841, 208.921145, -18.95004,  93.7305751, 212.713336};
      case calibmar::CameraModelType::OpenCVFisheyeCameraModel:
        return {105.345678, 116.318389, 127.340804, 87.1324962, -0.893005024, 2.89266999, -0.78731496, -1.12493278};
      default:
        throw std::runtime_error("Bad CameraModel");
    }
  }

  std::vector<Eigen::Vector3d> points3D = {
      {1.0, 1.0, 0.0}, {2.0, 1.0, 0.0}, {3.0, 1.0, 0.0}, {4.0, 1.0, 0.0}, {5.0, 1.0, 0.0}, {6.0, 1.0, 0.0}, {7.0, 1.0, 0.0},
      {8.0, 1.0, 0.0}, {9.0, 1.0, 0.0}, {1.0, 2.0, 0.0}, {2.0, 2.0, 0.0}, {3.0, 2.0, 0.0}, {4.0, 2.0, 0.0}, {5.0, 2.0, 0.0},
      {6.0, 2.0, 0.0}, {7.0, 2.0, 0.0}, {8.0, 2.0, 0.0}, {9.0, 2.0, 0.0}, {1.0, 3.0, 0.0}, {2.0, 3.0, 0.0}, {3.0, 3.0, 0.0},
      {4.0, 3.0, 0.0}, {5.0, 3.0, 0.0}, {6.0, 3.0, 0.0}, {7.0, 3.0, 0.0}, {8.0, 3.0, 0.0}, {9.0, 3.0, 0.0}, {1.0, 4.0, 0.0},
      {2.0, 4.0, 0.0}, {3.0, 4.0, 0.0}, {4.0, 4.0, 0.0}, {5.0, 4.0, 0.0}, {6.0, 4.0, 0.0}, {7.0, 4.0, 0.0}, {8.0, 4.0, 0.0},
      {9.0, 4.0, 0.0}, {1.0, 5.0, 0.0}, {2.0, 5.0, 0.0}, {3.0, 5.0, 0.0}, {4.0, 5.0, 0.0}, {5.0, 5.0, 0.0}, {6.0, 5.0, 0.0},
      {7.0, 5.0, 0.0}, {8.0, 5.0, 0.0}, {9.0, 5.0, 0.0}, {1.0, 6.0, 0.0}, {2.0, 6.0, 0.0}, {3.0, 6.0, 0.0}, {4.0, 6.0, 0.0},
      {5.0, 6.0, 0.0}, {6.0, 6.0, 0.0}, {7.0, 6.0, 0.0}, {8.0, 6.0, 0.0}, {9.0, 6.0, 0.0}};

  std::vector<std::vector<Eigen::Vector2d>> point2DSets = {
      {{1.02413338e+02, 5.45722580e+01}, {1.14660538e+02, 5.89496269e+01}, {1.26547035e+02, 6.31805878e+01},
       {1.37572189e+02, 6.71291428e+01}, {1.53330063e+02, 7.25110321e+01}, {1.63795868e+02, 7.58470306e+01},
       {1.72505020e+02, 7.91910706e+01}, {1.72504959e+02, 7.91911545e+01}, {1.81593826e+02, 8.22971725e+01},
       {1.01456291e+02, 6.75303802e+01}, {1.14568520e+02, 7.17654419e+01}, {1.26501137e+02, 7.56963425e+01},
       {1.37734985e+02, 7.94419556e+01}, {1.48781601e+02, 8.29746857e+01}, {1.63824417e+02, 8.76853485e+01},
       {1.74153748e+02, 9.11280975e+01}, {1.83007339e+02, 9.36844635e+01}, {1.83012924e+02, 9.36879959e+01},
       {1.00477135e+02, 8.14593964e+01}, {1.13521477e+02, 8.52960892e+01}, {1.26451714e+02, 8.91520462e+01},
       {1.38570923e+02, 9.26026611e+01}, {1.53963943e+02, 9.71985474e+01}, {1.64709824e+02, 1.00259354e+02},
       {1.74925217e+02, 1.03586029e+02}, {1.84749176e+02, 1.06125359e+02}, {1.84751144e+02, 1.06126587e+02},
       {9.92365570e+01, 9.64035416e+01}, {1.13183670e+02, 9.99727249e+01}, {1.26330650e+02, 1.03426437e+02},
       {1.38512329e+02, 1.06629089e+02}, {1.50502563e+02, 1.09752831e+02}, {1.66272110e+02, 1.13900009e+02},
       {1.77256958e+02, 1.16897324e+02}, {1.85796509e+02, 1.18969002e+02}, {1.94902237e+02, 1.21667152e+02},
       {9.77387085e+01, 1.12417320e+02}, {1.12468826e+02, 1.15681129e+02}, {1.26136841e+02, 1.18731102e+02},
       {1.38589569e+02, 1.21573944e+02}, {1.50683777e+02, 1.24429802e+02}, {1.67244629e+02, 1.28218124e+02},
       {1.77936432e+02, 1.30587433e+02}, {1.88034332e+02, 1.33020676e+02}, {1.88036560e+02, 1.33020340e+02},
       {9.65188828e+01, 1.29719940e+02}, {1.11584579e+02, 1.32516541e+02}, {1.25764946e+02, 1.35164322e+02},
       {1.39451965e+02, 1.37727249e+02}, {1.51451996e+02, 1.40173935e+02}, {1.63524078e+02, 1.42595093e+02},
       {1.79706558e+02, 1.45544022e+02}, {1.90661179e+02, 1.47865097e+02}, {1.98604813e+02, 1.49567932e+02}},
      {{1.30501740e+02, 6.98677750e+01}, {1.38433029e+02, 6.80634155e+01}, {1.47748413e+02, 6.57559738e+01},
       {1.64550644e+02, 6.21364021e+01}, {1.76566971e+02, 5.93783493e+01}, {1.89163834e+02, 5.62842178e+01},
       {2.02526566e+02, 5.31993866e+01}, {2.16766174e+02, 4.99585457e+01}, {2.32174393e+02, 4.66030159e+01},
       {1.37467468e+02, 8.00565948e+01}, {1.37468582e+02, 8.00557480e+01}, {1.49308136e+02, 7.78461838e+01},
       {1.64553970e+02, 7.49159851e+01}, {1.76607681e+02, 7.26172409e+01}, {1.89470093e+02, 7.01819763e+01},
       {2.03510956e+02, 6.73750000e+01}, {2.18100052e+02, 6.45223083e+01}, {2.33611923e+02, 6.15919342e+01},
       {1.29664642e+02, 9.38591766e+01}, {1.37593353e+02, 9.25102081e+01}, {1.53383728e+02, 9.01185837e+01},
       {1.64553848e+02, 8.83725815e+01}, {1.76961258e+02, 8.64226227e+01}, {1.90347046e+02, 8.43809509e+01},
       {2.04396179e+02, 8.22111588e+01}, {2.19382080e+02, 7.96949234e+01}, {2.35356628e+02, 7.72756882e+01},
       {1.35769531e+02, 1.05747864e+02}, {1.40252548e+02, 1.05309151e+02}, {1.52787216e+02, 1.03737465e+02},
       {1.64576660e+02, 1.02397591e+02}, {1.77242020e+02, 1.00925926e+02}, {1.90495667e+02, 9.93536148e+01},
       {2.05063965e+02, 9.76225433e+01}, {2.20526672e+02, 9.57443008e+01}, {2.37114410e+02, 9.37028580e+01},
       {1.28469193e+02, 1.20233353e+02}, {1.41431885e+02, 1.18873795e+02}, {1.52478897e+02, 1.17946808e+02},
       {1.64544998e+02, 1.17054649e+02}, {1.77443161e+02, 1.16034615e+02}, {1.91233307e+02, 1.14877525e+02},
       {2.05967484e+02, 1.13732193e+02}, {2.21681656e+02, 1.12512978e+02}, {2.38869995e+02, 1.11130898e+02},
       {1.29968643e+02, 1.33628372e+02}, {1.35244705e+02, 1.33459549e+02}, {1.52459717e+02, 1.32679459e+02},
       {1.64481705e+02, 1.32292496e+02}, {1.77508530e+02, 1.31768219e+02}, {1.91549423e+02, 1.31416367e+02},
       {2.07061218e+02, 1.30643066e+02}, {2.23378967e+02, 1.30187744e+02}, {2.40618134e+02, 1.29420212e+02}},
      {{1.03406906e+02, 7.07249527e+01}, {1.17409340e+02, 7.07086258e+01}, {1.30747055e+02, 7.06806488e+01},
       {1.44769058e+02, 7.06557541e+01}, {1.58736374e+02, 7.06522217e+01}, {1.70844162e+02, 7.02569809e+01},
       {1.83579666e+02, 7.02640457e+01}, {1.93834488e+02, 7.03866959e+01}, {1.93836990e+02, 7.03868179e+01},
       {1.05818977e+02, 8.07339859e+01}, {1.20608170e+02, 8.05619736e+01}, {1.34498871e+02, 8.03985214e+01},
       {1.47839142e+02, 8.01774368e+01}, {1.62219269e+02, 7.98422928e+01}, {1.74917023e+02, 7.94389038e+01},
       {1.88551071e+02, 7.90736847e+01}, {2.00488281e+02, 7.88576431e+01}, {2.09303894e+02, 7.87789688e+01},
       {1.08728271e+02, 9.20842438e+01}, {1.24188370e+02, 9.14421539e+01}, {1.38726761e+02, 9.08781204e+01},
       {1.52697556e+02, 9.04329071e+01}, {1.66289993e+02, 9.00943298e+01}, {1.80021347e+02, 8.94244232e+01},
       {1.95566193e+02, 8.87317963e+01}, {2.05660355e+02, 8.83956680e+01}, {2.05693237e+02, 8.83972855e+01},
       {1.11778625e+02, 1.04684502e+02}, {1.28163818e+02, 1.03692673e+02}, {1.43489258e+02, 1.02776634e+02},
       {1.58148056e+02, 1.02120865e+02}, {1.72044174e+02, 1.01262634e+02}, {1.85638565e+02, 1.00426628e+02},
       {2.00132187e+02, 9.95662994e+01}, {2.13777771e+02, 9.87192001e+01}, {2.24646820e+02, 9.82821732e+01},
       {1.15370323e+02, 1.19129593e+02}, {1.32643463e+02, 1.17663521e+02}, {1.48868469e+02, 1.16306389e+02},
       {1.64322479e+02, 1.15019012e+02}, {1.78815094e+02, 1.13823997e+02}, {1.92944885e+02, 1.12685234e+02},
       {2.06679352e+02, 1.11579338e+02}, {2.20737747e+02, 1.10456635e+02}, {2.28640854e+02, 1.09835129e+02},
       {1.19381210e+02, 1.35544067e+02}, {1.37768311e+02, 1.33347168e+02}, {1.55043594e+02, 1.31352386e+02},
       {1.71338333e+02, 1.29517120e+02}, {1.86741333e+02, 1.28129959e+02}, {2.01295563e+02, 1.26629654e+02},
       {2.15547501e+02, 1.24847198e+02}, {2.29266785e+02, 1.23439613e+02}, {2.43170120e+02, 1.22184334e+02}},
      {{1.04899330e+02, 4.39265251e+01}, {1.17170700e+02, 4.47173691e+01}, {1.30136963e+02, 4.56868095e+01},
       {1.43772156e+02, 4.65045052e+01}, {1.58520325e+02, 4.73628235e+01}, {1.74432938e+02, 4.83577881e+01},
       {1.91387375e+02, 4.91581573e+01}, {2.09541748e+02, 4.98568535e+01}, {2.28992584e+02, 5.06510773e+01},
       {1.07567741e+02, 5.89902077e+01}, {1.19588829e+02, 6.02735138e+01}, {1.32130539e+02, 6.14618607e+01},
       {1.45467484e+02, 6.27091370e+01}, {1.59528839e+02, 6.40006332e+01}, {1.74466324e+02, 6.53417969e+01},
       {1.90555130e+02, 6.67372894e+01}, {2.07745544e+02, 6.82489014e+01}, {2.26473450e+02, 6.96376953e+01},
       {1.10257790e+02, 7.30581207e+01}, {1.21686188e+02, 7.45890732e+01}, {1.33755188e+02, 7.61454086e+01},
       {1.46487656e+02, 7.77074738e+01}, {1.60208588e+02, 7.93684616e+01}, {1.74527390e+02, 8.11354370e+01},
       {1.90097504e+02, 8.29339523e+01}, {2.06502884e+02, 8.49395905e+01}, {2.24174545e+02, 8.70808029e+01},
       {1.09367210e+02, 8.54565277e+01}, {1.18658630e+02, 8.70652390e+01}, {1.35271103e+02, 8.97010117e+01},
       {1.47564667e+02, 9.15432281e+01}, {1.60464996e+02, 9.35749969e+01}, {1.74530411e+02, 9.57127533e+01},
       {1.89464233e+02, 9.80011597e+01}, {2.05242432e+02, 1.00374619e+02}, {2.22031265e+02, 1.02943016e+02},
       {1.20353912e+02, 9.88369064e+01}, {1.20354721e+02, 9.88375854e+01}, {1.36243240e+02, 1.02129608e+02},
       {1.48502289e+02, 1.04439110e+02}, {1.61429001e+02, 1.06711334e+02}, {1.74521057e+02, 1.09188972e+02},
       {1.88825500e+02, 1.11644806e+02}, {2.04032898e+02, 1.14477135e+02}, {2.20184113e+02, 1.17390442e+02},
       {1.13385239e+02, 1.08495750e+02}, {1.22336266e+02, 1.10721611e+02}, {1.32474045e+02, 1.12904892e+02},
       {1.49511444e+02, 1.16432884e+02}, {1.61606613e+02, 1.18794106e+02}, {1.74638702e+02, 1.21427940e+02},
       {1.88464645e+02, 1.24498695e+02}, {2.02801498e+02, 1.27468079e+02}, {2.18376572e+02, 1.30620422e+02}}};
}