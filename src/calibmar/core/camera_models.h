#pragma once

#include <colmap/scene/camera.h>
#include <colmap/sensor/models.h>
#include <colmap/sensor/models_refrac.h>
#include <string>
#include <vector>

namespace calibmar {

  enum class CameraModelType {
    // f, cx, cy
    SimplePinholeCameraModel,
    // fx, fy, cx, cy
    PinholeCameraModel,
    // f, cx, cy, k
    SimpleRadialCameraModel,
    // f, cx, cy, k1, k2
    RadialCameraModel,
    // fx, fy, cx, cy, k1, k2, p1, p2
    OpenCVCameraModel,
    // fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
    FullOpenCVCameraModel,
    // fx, fy, cx, cy, k1, k2, k3, k4
    OpenCVFisheyeCameraModel
  };

  enum class HousingInterfaceType {
    // Cx, Cy, Cz, int_radius, int_thick, na, ng, nw
    DoubleLayerSphericalRefractive,
    // Nx, Ny, Nz, int_dist, int_thick, na, ng, nw
    DoubleLayerPlanarRefractive
  };

  // Util class holding camera model information
  struct CameraModel {
    std::string friendly_name;
    std::string model_name;
    std::string params_info;
    size_t num_params;

    static inline colmap::Camera InitCamera(CameraModelType type, const std::pair<int, int>& image_size,
                                            const std::vector<double>& params) {
      colmap::Camera camera = InitCamera(type, image_size, 0);
      camera.params = params;
      return camera;
    }

    static inline colmap::Camera InitCamera(CameraModelType type, const std::pair<int, int>& image_size, double focal_length) {
      colmap::Camera camera = colmap::Camera::CreateFromModelName(colmap::kInvalidCameraId, CameraModels().at(type).model_name,
                                                                  focal_length, image_size.first, image_size.second);
      return camera;
    }

    // Maps CameraModelType to CameraModel
    static inline std::map<CameraModelType, CameraModel> CameraModels() {
      return {{CameraModelType::SimplePinholeCameraModel,
               {"Simple Pinhole", colmap::SimplePinholeCameraModel::model_name, colmap::SimplePinholeCameraModel::params_info,
                colmap::SimplePinholeCameraModel::num_params}},
              {CameraModelType::PinholeCameraModel,
               {"Pinhole", colmap::PinholeCameraModel::model_name, colmap::PinholeCameraModel::params_info,
                colmap::PinholeCameraModel::num_params}},
              {CameraModelType::SimpleRadialCameraModel,
               {"Simple Radial", colmap::SimpleRadialCameraModel::model_name, colmap::SimpleRadialCameraModel::params_info,
                colmap::SimpleRadialCameraModel::num_params}},
              {CameraModelType::RadialCameraModel,
               {"Radial", colmap::RadialCameraModel::model_name, colmap::RadialCameraModel::params_info,
                colmap::RadialCameraModel::num_params}},
              {CameraModelType::OpenCVCameraModel,
               {"OpenCV", colmap::OpenCVCameraModel::model_name, colmap::OpenCVCameraModel::params_info,
                colmap::OpenCVCameraModel::num_params}},
              {CameraModelType::FullOpenCVCameraModel,
               {"Full OpenCV", colmap::FullOpenCVCameraModel::model_name, colmap::FullOpenCVCameraModel::params_info,
                colmap::FullOpenCVCameraModel::num_params}},
              {CameraModelType::OpenCVFisheyeCameraModel,
               {"OpenCV Fisheye", colmap::OpenCVFisheyeCameraModel::model_name, colmap::OpenCVFisheyeCameraModel::params_info,
                colmap::OpenCVFisheyeCameraModel::num_params}}};
    }

    static inline std::map<colmap::CameraModelId, CameraModelType> IdToCameraModelType() {
      return {{colmap::SimplePinholeCameraModel::model_id, CameraModelType::SimplePinholeCameraModel},
              {colmap::PinholeCameraModel::model_id, CameraModelType::PinholeCameraModel},
              {colmap::SimpleRadialCameraModel::model_id, CameraModelType::SimpleRadialCameraModel},
              {colmap::RadialCameraModel::model_id, CameraModelType::RadialCameraModel},
              {colmap::OpenCVCameraModel::model_id, CameraModelType::OpenCVCameraModel},
              {colmap::FullOpenCVCameraModel::model_id, CameraModelType::FullOpenCVCameraModel},
              {colmap::OpenCVFisheyeCameraModel::model_id, CameraModelType::OpenCVFisheyeCameraModel}};
    }

    static inline bool IsFisheyeModel(colmap::CameraModelId colmap_model_id) {
      switch (colmap_model_id) {
        case colmap::MetashapeFisheyeCameraModel::model_id:
        case colmap::OpenCVFisheyeCameraModel::model_id:
        case colmap::RadialFisheyeCameraModel::model_id:
        case colmap::SimpleRadialFisheyeCameraModel::model_id:
        case colmap::ThinPrismFisheyeCameraModel::model_id:
          return true;
        default:
          return false;
      }
    }
  };

  // Util class holding housing interface information
  struct HousingInterface {
    std::string friendly_name;
    std::string model_name;
    std::string params_info;
    size_t num_params;

    static inline std::map<HousingInterfaceType, HousingInterface> HousingInterfaces() {
      return {{HousingInterfaceType::DoubleLayerSphericalRefractive,
               {"Thick Dome Port", colmap::DomePort::refrac_model_name, colmap::DomePort::refrac_params_info,
                colmap::DomePort::num_params}},
              {HousingInterfaceType::DoubleLayerPlanarRefractive,
               {"Thick Flat Port", colmap::FlatPort::refrac_model_name, colmap::FlatPort::refrac_params_info,
                colmap::FlatPort::num_params}}};
    }
  };
}