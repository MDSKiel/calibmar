
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
  camera.model_id = colmap::CameraModelNameToId(model_name);
  camera.refrac_model_id = colmap::CameraRefracModelNameToId(housing_name);
  camera.width = 1000;
  camera.height = 2000;
  camera.params = {1.1, -2, 3, 4, 5, 6, 7, 8};
  camera.refrac_params = {1, 2, 3, 4, 5, 6, 7, 8};
  calibration.SetCamera(camera);
  std::stringstream string;
  report::GenerateCalibrationYaml(string, calibration);
  std::string yaml_string = string.str();

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
BOOST_TEST_DONT_PRINT_LOG_VALUE(CalibrationTargetType)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ArucoMarkerTypes)
BOOST_AUTO_TEST_CASE(CameraYAML_Can_Import_Export) {
  Calibration calibration;
  colmap::Camera camera;
  std::string model_name = CameraModel::CameraModels().at(CameraModelType::OpenCVCameraModel).model_name;
  std::string housing_name =
      HousingInterface::HousingInterfaces().at(HousingInterfaceType::DoubleLayerSphericalRefractive).model_name;
  camera.model_id = colmap::CameraModelNameToId(model_name);
  camera.refrac_model_id = colmap::CameraRefracModelNameToId(housing_name);
  camera.width = 1000;
  camera.height = 2000;
  camera.params = {1.1, -2, 3, 4, 5, 6, 7, 8};
  camera.refrac_params = {1, 2, 3, 4, 5, 6, 7, 8};
  calibration.SetCamera(camera);
  // intentionally a constant string
  calibration.SetCalibrationTargetInfo("chessboard, 10, 7, 0.04");

  std::stringstream string;
  report::GenerateCalibrationYaml(string, calibration);
  string.seekg(0);
  ImportedParameters parameters = ImportedParameters::ImportFromYaml(string);

  // the exported yaml should be importable
  BOOST_TEST(parameters.camera_model == CameraModelType::OpenCVCameraModel);
  BOOST_TEST(parameters.camera_parameters.size() == camera.params.size());
  for (size_t i = 0; i < parameters.camera_parameters.size(); i++) {
    BOOST_TEST(parameters.camera_parameters[i] == camera.params[i]);
  }
  BOOST_TEST(parameters.columns == 10);
  BOOST_TEST(parameters.rows == 7);
  BOOST_TEST(parameters.housing_model.value() == HousingInterfaceType::DoubleLayerSphericalRefractive);
  BOOST_TEST(parameters.housing_parameters.size() == camera.refrac_params.size());
  for (size_t i = 0; i < parameters.housing_parameters.size(); i++) {
    BOOST_TEST(parameters.housing_parameters[i] == camera.refrac_params[i]);
  }
  BOOST_TEST(parameters.square_size == 0.04);
  BOOST_TEST(parameters.calibration_target == CalibrationTargetType::Chessboard);
}

BOOST_AUTO_TEST_CASE(CameraYAML_Import_Export_3DTargetAruco) {
  Calibration calibration;
  colmap::Camera camera;
  std::string model_name = CameraModel::CameraModels().at(CameraModelType::OpenCVCameraModel).model_name;
  camera.model_id = colmap::CameraModelNameToId(model_name);
  calibration.SetCamera(camera);
  // intentionally a constant string
  calibration.SetCalibrationTargetInfo("3D target, aruco: 4x4_50, 3.2");

  std::stringstream string;
  report::GenerateCalibrationYaml(string, calibration);
  string.seekg(0);
  ImportedParameters parameters = ImportedParameters::ImportFromYaml(string);

  // the exported yaml should be importable
  BOOST_TEST(parameters.calibration_target == CalibrationTargetType::Target3DAruco);
  BOOST_TEST(parameters.aruco_type == ArucoMarkerTypes::DICT_4X4_50);
  BOOST_TEST(parameters.aruco_scale_factor == 3.2);
}

BOOST_AUTO_TEST_CASE(CameraYAML_Import_Export_3DTarget) {
  Calibration calibration;
  colmap::Camera camera;
  std::string model_name = CameraModel::CameraModels().at(CameraModelType::OpenCVCameraModel).model_name;
  camera.model_id = colmap::CameraModelNameToId(model_name);
  calibration.SetCamera(camera);
  // intentionally a constant string
  calibration.SetCalibrationTargetInfo("3D target");

  std::stringstream string;
  report::GenerateCalibrationYaml(string, calibration);
  string.seekg(0);
  ImportedParameters parameters = ImportedParameters::ImportFromYaml(string);

  // the exported yaml should be importable
  BOOST_TEST(parameters.calibration_target == CalibrationTargetType::Target3D);
}

BOOST_TEST_DONT_PRINT_LOG_VALUE(ArucoGridOrigin)
BOOST_TEST_DONT_PRINT_LOG_VALUE(ArucoGridDirection)
BOOST_AUTO_TEST_CASE(CameraYAML_Import_Export_3DTargetArucoGridBoard) {
  Calibration calibration;
  colmap::Camera camera;
  std::string model_name = CameraModel::CameraModels().at(CameraModelType::OpenCVCameraModel).model_name;
  camera.model_id = colmap::CameraModelNameToId(model_name);
  calibration.SetCamera(camera);
  // intentionally a constant string
  calibration.SetCalibrationTargetInfo("aruco grid board, 6X6_100, 5, 7, 0.026, 0.003, 2, bl, hor");

  std::stringstream string;
  report::GenerateCalibrationYaml(string, calibration);
  string.seekg(0);
  ImportedParameters parameters = ImportedParameters::ImportFromYaml(string);

  // the exported yaml should be importable
  BOOST_TEST(parameters.calibration_target == CalibrationTargetType::ArucoGridBoard);
  BOOST_TEST(parameters.aruco_type == ArucoMarkerTypes::DICT_6X6_100);
  BOOST_TEST(parameters.columns == 5);
  BOOST_TEST(parameters.rows == 7);
  BOOST_TEST(parameters.square_size == 0.026);
  BOOST_TEST(parameters.spacing == 0.003);
  BOOST_TEST(parameters.grid_origin == ArucoGridOrigin::BottomLeft);
  BOOST_TEST(parameters.grid_direction == ArucoGridDirection::Horizontal);
  BOOST_TEST(parameters.border_bits == 2);
}
