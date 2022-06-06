// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "base/camera_non_svp_models.h"

#include <unordered_map>

namespace colmap {

// Initialize params_info, model_name, num_params, model_id, etc.

#define CAMERA_NON_SVP_MODEL_CASE(CameraNonSvpModel)                         \
  const int CameraNonSvpModel::non_svp_model_id = InitializeNonSvpModelId(); \
  const std::string CameraNonSvpModel::non_svp_model_name =                  \
      CameraNonSvpModel::InitializeNonSvpModelName();                        \
  const size_t CameraNonSvpModel::num_params = InitializeNumParams();        \
  const std::string CameraNonSvpModel::params_info =                         \
      CameraNonSvpModel::InitializeNonSvpModelParamsInfo();

CAMERA_NON_SVP_MODEL_CASES

#undef CAMERA_NON_SVP_MODEL_CASE

std::unordered_map<std::string, int> InitializeCameraNonSvpModelNameToId() {
  std::unordered_map<std::string, int> camera_non_svp_model_name_to_id;

#define CAMERA_NON_SVP_MODEL_CASE(CameraNonSvpModel) \
  camera_non_svp_model_name_to_id.emplace(           \
      CameraNonSvpModel::non_svp_model_name,         \
      CameraNonSvpModel::non_svp_model_id);

  CAMERA_NON_SVP_MODEL_CASES

#undef CAMERA_NON_SVP_MODEL_CASE
  return camera_non_svp_model_name_to_id;
}

std::unordered_map<int, std::string> InitializeCameraNonSvpModelIdToName() {
  std::unordered_map<int, std::string> camera_non_svp_model_id_to_name;

#define CAMERA_NON_SVP_MODEL_CASE(CameraNonSvpModel) \
  camera_non_svp_model_id_to_name.emplace(           \
      CameraNonSvpModel::non_svp_model_id,           \
      CameraNonSvpModel::non_svp_model_name);

  CAMERA_NON_SVP_MODEL_CASES

#undef CAMERA_NON_SVP_MODEL_CASE
  return camera_non_svp_model_id_to_name;
}

static const std::unordered_map<std::string, int>
    CAMERA_NON_SVP_MODEL_NAME_TO_ID = InitializeCameraNonSvpModelNameToId();

static const std::unordered_map<int, std::string>
    CAMERA_NON_SVP_MODEL_ID_TO_NAME = InitializeCameraNonSvpModelIdToName();

bool ExistsCameraNonSvpModelWithName(const std::string& non_svp_model_name) {
  return CAMERA_NON_SVP_MODEL_NAME_TO_ID.count(non_svp_model_name) > 0;
}

bool ExistsCameraNonSvpModelWithId(const int non_svp_model_id) {
  return CAMERA_NON_SVP_MODEL_ID_TO_NAME.count(non_svp_model_id) > 0;
}

int CameraNonSvpModelNameToId(const std::string& non_svp_model_name) {
  const auto it = CAMERA_NON_SVP_MODEL_NAME_TO_ID.find(non_svp_model_name);
  if (it == CAMERA_NON_SVP_MODEL_NAME_TO_ID.end()) {
    return kInvalidCameraNonSvpModelId;
  } else {
    return it->second;
  }
}

std::string CameraNonSvpModelIdToName(const int non_svp_model_id) {
  const auto it = CAMERA_NON_SVP_MODEL_ID_TO_NAME.find(non_svp_model_id);
  if (it == CAMERA_NON_SVP_MODEL_ID_TO_NAME.end()) {
    return "";
  } else {
    return it->second;
  }
}

std::string CameraNonSvpModelParamsInfo(const int non_svp_model_id) {
  switch (non_svp_model_id) {
#define CAMERA_NON_SVP_MODEL_CASE(CameraNonSvpModel) \
  case CameraNonSvpModel::kNonSvpModelId:            \
    return CameraNonSvpModel::params_info;           \
    break;

    CAMERA_NON_SVP_MODEL_SWITCH_CASES

#undef CAMERA_NON_SVP_MODEL_CASE
  }
  return "Camera Non-Single-View-Point model does not exist";
}

size_t CameraNonSvpModelNumParams(const int non_svp_model_id) {
  switch (non_svp_model_id) {
#define CAMERA_NON_SVP_MODEL_CASE(CameraNonSvpModel) \
  case CameraNonSvpModel::kNonSvpModelId:            \
    return CameraNonSvpModel::num_params;            \
    break;

    CAMERA_NON_SVP_MODEL_SWITCH_CASES

#undef CAMERA_NON_SVP_MODEL_CASE
  }
  return 0;
}

bool CameraNonSvpModelVerifyParams(const int non_svp_model_id,
                                   const std::vector<double>& params) {
  switch (non_svp_model_id) {
#define CAMERA_NON_SVP_MODEL_CASE(CameraNonSvpModel)      \
  case CameraNonSvpModel::kNonSvpModelId:                 \
    if (params.size() == CameraNonSvpModel::num_params) { \
      return true;                                        \
    }                                                     \
    break;

    CAMERA_NON_SVP_MODEL_SWITCH_CASES

#undef CAMERA_NON_SVP_MODEL_CASE
  }
  return false;
}

}  // namespace colmap