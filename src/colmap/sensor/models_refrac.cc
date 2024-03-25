#include "colmap/sensor/models_refrac.h"

#include <unordered_map>

namespace colmap {

// Initialize params_info, model_name, num_params, model_id, etc.

#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)                      \
  constexpr CameraRefracModelId CameraRefracModel::refrac_model_id;      \
  const std::string CameraRefracModel::refrac_model_name =               \
      CameraRefracModel::InitializeRefracModelName();                    \
  constexpr size_t CameraRefracModel::num_params;                        \
  const std::string CameraRefracModel::refrac_params_info =              \
      CameraRefracModel::InitializeRefracParamsInfo();                   \
  const std::vector<size_t> CameraRefracModel::optimizable_params_idxs = \
      CameraRefracModel::InitializeOptimizableParamsIdxs();

CAMERA_REFRAC_MODEL_CASES

#undef CAMERA_REFRAC_MODEL_CASE

std::unordered_map<std::string, CameraRefracModelId>
InitializeCameraRefracModelNameToId() {
  std::unordered_map<std::string, CameraRefracModelId>
      camera_refrac_model_name_to_id;

#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)                            \
  camera_refrac_model_name_to_id.emplace(CameraRefracModel::refrac_model_name, \
                                         CameraRefracModel::refrac_model_id);

  CAMERA_REFRAC_MODEL_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  return camera_refrac_model_name_to_id;
}

std::unordered_map<CameraRefracModelId, const std::string*>
InitializeCameraRefracModelIdToName() {
  std::unordered_map<CameraRefracModelId, const std::string*>
      camera_refrac_model_id_to_name;

#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel) \
  camera_refrac_model_id_to_name.emplace(           \
      CameraRefracModel::refrac_model_id,           \
      &CameraRefracModel::refrac_model_name);

  CAMERA_REFRAC_MODEL_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  return camera_refrac_model_id_to_name;
}

static const std::unordered_map<std::string, CameraRefracModelId>
    kCameraRefracModelNameToId = InitializeCameraRefracModelNameToId();

static const std::unordered_map<CameraRefracModelId, const std::string*>
    kCameraRefracModelIdToName = InitializeCameraRefracModelIdToName();

bool ExistsCameraRefracModelWithName(const std::string& refrac_model_name) {
  return kCameraRefracModelNameToId.count(refrac_model_name) > 0;
}

bool ExistsCameraRefracModelWithId(const CameraRefracModelId refrac_model_id) {
  return kCameraRefracModelIdToName.count(refrac_model_id) > 0;
}

CameraRefracModelId CameraRefracModelNameToId(
    const std::string& refrac_model_name) {
  const auto it = kCameraRefracModelNameToId.find(refrac_model_name);
  if (it == kCameraRefracModelNameToId.end()) {
    return CameraRefracModelId::kInvalid;
  } else {
    return it->second;
  }
}

const std::string& CameraRefracModelIdToName(
    const CameraRefracModelId refrac_model_id) {
  const auto it = kCameraRefracModelIdToName.find(refrac_model_id);
  if (it == kCameraRefracModelIdToName.end()) {
    const static std::string kEmptyRefracModelName = "";
    return kEmptyRefracModelName;
  } else {
    return *(it->second);
  }
}

const std::string& CameraRefracModelParamsInfo(
    const CameraRefracModelId refrac_model_id) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel) \
  case CameraRefracModel::refrac_model_id:          \
    return CameraRefracModel::refrac_params_info;   \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }

  const static std::string kEmptyParamsInfo = "";
  return kEmptyParamsInfo;
}

const std::vector<size_t>& CameraRefracModelOptimizableParamsIdxs(
    CameraRefracModelId refrac_model_id) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)    \
  case CameraRefracModel::refrac_model_id:             \
    return CameraRefracModel::optimizable_params_idxs; \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }

  static const std::vector<size_t> kEmptyIdxs;
  return kEmptyIdxs;
}

size_t CameraRefracModelNumParams(const CameraRefracModelId refrac_model_id) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel) \
  case CameraRefracModel::refrac_model_id:          \
    return CameraRefracModel::num_params;           \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }

  return 0;
}

bool CameraRefracModelVerifyParams(const CameraRefracModelId refrac_model_id,
                                   const std::vector<double>& params) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)       \
  case CameraRefracModel::refrac_model_id:                \
    if (params.size() == CameraRefracModel::num_params) { \
      return true;                                        \
    }                                                     \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }

  return false;
}

}  // namespace colmap