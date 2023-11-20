#include "colmap/sensor/models_refrac.h"

#include <unordered_map>

namespace colmap {

// Initialize params_info, model_name, num_params, model_id, etc.

#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)                         \
  const int CameraRefracModel::refrac_model_id = InitializeRefracModelId(); \
  const std::string CameraRefracModel::refrac_model_name =                  \
      CameraRefracModel::InitializeRefracModelName();                       \
  const size_t CameraRefracModel::num_params = InitializeNumParams();       \
  const std::string CameraRefracModel::params_info =                        \
      CameraRefracModel::InitializeRefracModelParamsInfo();                 \
  const std::vector<size_t> CameraRefracModel::optimizable_params_idxs =    \
      CameraRefracModel::InitializeOptimizableParamsIdxs();

CAMERA_REFRAC_MODEL_CASES

#undef CAMERA_REFRAC_MODEL_CASE

std::unordered_map<std::string, int> InitializeCameraRefracModelNameToId() {
  std::unordered_map<std::string, int> camera_refrac_model_name_to_id;

#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)                            \
  camera_refrac_model_name_to_id.emplace(CameraRefracModel::refrac_model_name, \
                                         CameraRefracModel::refrac_model_id);

  CAMERA_REFRAC_MODEL_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  return camera_refrac_model_name_to_id;
}

std::unordered_map<int, std::string> InitializeCameraRefracModelIdToName() {
  std::unordered_map<int, std::string> camera_refrac_model_id_to_name;

#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel) \
  camera_refrac_model_id_to_name.emplace(           \
      CameraRefracModel::refrac_model_id,           \
      CameraRefracModel::refrac_model_name);

  CAMERA_REFRAC_MODEL_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  return camera_refrac_model_id_to_name;
}

static const std::unordered_map<std::string, int>
    CAMERA_REFRAC_MODEL_NAME_TO_ID = InitializeCameraRefracModelNameToId();

static const std::unordered_map<int, std::string>
    CAMERA_REFRAC_MODEL_ID_TO_NAME = InitializeCameraRefracModelIdToName();

bool ExistsCameraRefracModelWithName(const std::string& refrac_model_name) {
  return CAMERA_REFRAC_MODEL_NAME_TO_ID.count(refrac_model_name) > 0;
}

bool ExistsCameraRefracModelWithId(const int refrac_model_id) {
  return CAMERA_REFRAC_MODEL_ID_TO_NAME.count(refrac_model_id) > 0;
}

int CameraRefracModelNameToId(const std::string& refrac_model_name) {
  const auto it = CAMERA_REFRAC_MODEL_NAME_TO_ID.find(refrac_model_name);
  if (it == CAMERA_REFRAC_MODEL_NAME_TO_ID.end()) {
    return kInvalidRefractiveCameraModelId;
  } else {
    return it->second;
  }
}

std::string CameraRefracModelIdToName(const int refrac_model_id) {
  const auto it = CAMERA_REFRAC_MODEL_ID_TO_NAME.find(refrac_model_id);
  if (it == CAMERA_REFRAC_MODEL_ID_TO_NAME.end()) {
    return "";
  } else {
    return it->second;
  }
}

std::string CameraRefracModelParamsInfo(const int refrac_model_id) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel) \
  case CameraRefracModel::kRefracModelId:           \
    return CameraRefracModel::params_info;          \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }
  return "Refractive camera model does not exist";
}

static const std::vector<size_t> EMPTY_IDXS;

const std::vector<size_t>& CameraRefracModelOptimizableParamsIdxs(
    int refrac_model_id) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)    \
  case CameraRefracModel::kRefracModelId:              \
    return CameraRefracModel::optimizable_params_idxs; \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }
  return EMPTY_IDXS;
}

size_t CameraRefracModelNumParams(const int refrac_model_id) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel) \
  case CameraRefracModel::kRefracModelId:           \
    return CameraRefracModel::num_params;           \
    break;

    CAMERA_REFRAC_MODEL_SWITCH_CASES

#undef CAMERA_REFRAC_MODEL_CASE
  }
  return 0;
}

bool CameraRefracModelVerifyParams(const int refrac_model_id,
                                   const std::vector<double>& params) {
  switch (refrac_model_id) {
#define CAMERA_REFRAC_MODEL_CASE(CameraRefracModel)       \
  case CameraRefracModel::kRefracModelId:                 \
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