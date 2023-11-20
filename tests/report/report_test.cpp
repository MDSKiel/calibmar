
#include "calibmar/core/camera_models.h"
#include "calibmar/core/report.h"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace calibmar;

BOOST_AUTO_TEST_CASE(CameraYAML_Structure) {
  Calibration calibration;
  colmap::Camera camera;
  std::string model_name = CameraModel::CameraModels().at(CameraModelType::OpenCVCameraModel).model_name;
  std::string housing_name =
      HousingInterface::HousingInterfaces().at(HousingInterfaceType::DoubleLayerSphericalRefractive).model_name;
  camera.SetModelIdFromName(model_name);
  camera.SetRefracModelIdFromName(housing_name);
  camera.SetWidth(1000);
  camera.SetHeight(2000);
  camera.SetParams({1.1, -2, 3, 4, 5, 6, 7, 8});
  camera.SetRefracParams({1, 2, 3, 4, 5, 6, 7, 8});
  calibration.SetCamera(camera);
  std::stringstream string;
  report::GenerateCalibrationYaml(string, calibration);
  std::string yaml_string = string.str();

  std::cout << yaml_string;

  // the exported yaml should adhere to this structure
  BOOST_TEST(yaml_string.find("model: " + model_name) != std::string::npos);
  BOOST_TEST(yaml_string.find("parameters: [1.1, -2, 3, 4, 5, 6, 7, 8]") != std::string::npos);
  BOOST_TEST(yaml_string.find("non_svp_model: " + housing_name) != std::string::npos);
  BOOST_TEST(yaml_string.find("non_svp_parameters: [1, 2, 3, 4, 5, 6, 7, 8]") != std::string::npos);
  BOOST_TEST(yaml_string.find("width: 1000") != std::string::npos);
  BOOST_TEST(yaml_string.find("height: 2000") != std::string::npos);
}

BOOST_TEST_DONT_PRINT_LOG_VALUE(CameraModelType)
BOOST_TEST_DONT_PRINT_LOG_VALUE(HousingInterfaceType)
BOOST_AUTO_TEST_CASE(CameraYAML_Can_Import_Export) {
  Calibration calibration;
  colmap::Camera camera;
  std::string model_name = CameraModel::CameraModels().at(CameraModelType::OpenCVCameraModel).model_name;
  std::string housing_name =
      HousingInterface::HousingInterfaces().at(HousingInterfaceType::DoubleLayerSphericalRefractive).model_name;
  camera.SetModelIdFromName(model_name);
  camera.SetRefracModelIdFromName(housing_name);
  camera.SetWidth(1000);
  camera.SetHeight(2000);
  camera.SetParams({1.1, -2, 3, 4, 5, 6, 7, 8});
  camera.SetRefracParams({1, 2, 3, 4, 5, 6, 7, 8});
  calibration.SetCamera(camera);
  calibration.SetCalibrationTargetInfo("chessboard, 10, 7, 0.04");

  std::stringstream string;
  report::GenerateCalibrationYaml(string, calibration);
  string.seekg(0);
  ImportedParameters parameters = ImportedParameters::ImportFromYaml(string);

  // the exported yaml should be importable
  BOOST_TEST(parameters.camera_model == CameraModelType::OpenCVCameraModel);
  BOOST_TEST(parameters.camera_parameters.size() == camera.Params().size());
  for (size_t i = 0; i < parameters.camera_parameters.size(); i++) {
    BOOST_TEST(parameters.camera_parameters[i] == camera.Params()[i]);
  }
  BOOST_TEST(parameters.chessboard_columns == 10);
  BOOST_TEST(parameters.chessboard_rows == 7);
  BOOST_TEST(parameters.housing_model.value() == HousingInterfaceType::DoubleLayerSphericalRefractive);
  BOOST_TEST(parameters.housing_parameters.size() == camera.RefracParams().size());
  for (size_t i = 0; i < parameters.housing_parameters.size(); i++) {
    BOOST_TEST(parameters.housing_parameters[i] == camera.RefracParams()[i]);
  }
  BOOST_TEST(parameters.square_size == 0.04);
}
