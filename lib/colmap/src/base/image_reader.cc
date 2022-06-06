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

#include <boost/filesystem/fstream.hpp>

#include "base/camera_models.h"
#include "base/camera_non_svp_models.h"
#include "base/image_reader.h"
#include "base/pose.h"
#include "util/misc.h"

#ifdef GDAL_ENABLED
#include "util/geo_conversion.cc"
#endif

namespace colmap {

bool ImageReaderOptions::Check() const {
  CHECK_OPTION_GT(default_focal_length_factor, 0.0);
  CHECK_OPTION(ExistsCameraModelWithName(camera_model));
  const int model_id = CameraModelNameToId(camera_model);
  if (!camera_params.empty()) {
    CHECK_OPTION(
        CameraModelVerifyParams(model_id, CSVToVector<double>(camera_params)));
  }
  if (camera_non_svp_model != "NONE") {
    CHECK_OPTION(ExistsCameraNonSvpModelWithName(camera_non_svp_model));
    CHECK_OPTION(!camera_non_svp_params.empty());
    const int non_svp_model_id =
        CameraNonSvpModelNameToId(camera_non_svp_model);

    CHECK_OPTION(CameraNonSvpModelVerifyParams(
        non_svp_model_id, CSVToVector<double>(camera_non_svp_params)));
  }
  
  return true;
}

ImageReader::ImageReader(const ImageReaderOptions& options, Database* database)
    : options_(options),
      database_(database),
      image_index_(0),
      global_tvec_prior_offset_(-999999.9, -999999.9, -999999.9) {
  CHECK(options_.Check());

  // Ensure trailing slash, so that we can build the correct image name.
  options_.image_path =
      EnsureTrailingSlash(StringReplace(options_.image_path, "\\", "/"));

  // Get a list of all files in the image path, sorted by image name.
  if (options_.image_list.empty()) {
    options_.image_list = GetRecursiveFileList(options_.image_path);
    std::sort(options_.image_list.begin(), options_.image_list.end());
  } else {
    if (!std::is_sorted(options_.image_list.begin(),
                        options_.image_list.end())) {
      std::sort(options_.image_list.begin(), options_.image_list.end());
    }
    for (auto& image_name : options_.image_list) {
      image_name = JoinPaths(options_.image_path, image_name);
    }
  }

  if (static_cast<camera_t>(options_.existing_camera_id) != kInvalidCameraId) {
    CHECK(database->ExistsCamera(options_.existing_camera_id));
    prev_camera_ = database->ReadCamera(options_.existing_camera_id);
  } else {
    // Set the manually specified camera parameters.
    prev_camera_.SetCameraId(kInvalidCameraId);
    prev_camera_.SetModelIdFromName(options_.camera_model);
    if (!options_.camera_params.empty()) {
      CHECK(prev_camera_.SetParamsFromString(options_.camera_params));
      prev_camera_.SetPriorFocalLength(true);
    }
    if (options_.camera_non_svp_model != "NONE") {
      prev_camera_.SetNonSvpModelIdFromName(options_.camera_non_svp_model);
      if (!options_.camera_non_svp_params.empty()) {
        CHECK(prev_camera_.SetNonSvpParamsFromString(
            options_.camera_non_svp_params));
      }
    }
  }
}

ImageReader::Status ImageReader::Next(Camera* camera, Image* image,
                                      Bitmap* bitmap, Bitmap* mask) {
  CHECK_NOTNULL(camera);
  CHECK_NOTNULL(image);
  CHECK_NOTNULL(bitmap);

  image_index_ += 1;
  CHECK_LE(image_index_, options_.image_list.size());

  const std::string image_path = options_.image_list.at(image_index_ - 1);

  DatabaseTransaction database_transaction(database_);

  //////////////////////////////////////////////////////////////////////////////
  // Set the image name.
  //////////////////////////////////////////////////////////////////////////////

  image->SetName(image_path);
  image->SetName(StringReplace(image->Name(), "\\", "/"));
  image->SetName(
      image->Name().substr(options_.image_path.size(),
                           image->Name().size() - options_.image_path.size()));

  const std::string image_folder = GetParentDir(image->Name());

  //////////////////////////////////////////////////////////////////////////////
  // Check if there's a csv with the same name as the image.
  //////////////////////////////////////////////////////////////////////////////

  bool have_csv_file = false;
  std::string image_path_stem, ext;
  SplitFileExtension(image_path, &image_path_stem, &ext);
  const std::string csv_path = image_path_stem + ".csv";
  if (csv_path != image_path && ExistsFile(csv_path)) have_csv_file = true;

  //////////////////////////////////////////////////////////////////////////////
  // Check if image already read.
  //////////////////////////////////////////////////////////////////////////////

  const bool exists_image = database_->ExistsImageWithName(image->Name());

  if (exists_image) {
    *image = database_->ReadImageWithName(image->Name());
    const bool exists_keypoints = database_->ExistsKeypoints(image->ImageId());
    const bool exists_descriptors =
        database_->ExistsDescriptors(image->ImageId());

    if (exists_keypoints && exists_descriptors) {
      return Status::IMAGE_EXISTS;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Read image.
  //////////////////////////////////////////////////////////////////////////////

  if (!bitmap->Read(image_path, false)) {
    return Status::BITMAP_ERROR;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Read mask.
  //////////////////////////////////////////////////////////////////////////////

  if (mask && !options_.mask_path.empty()) {
    const std::string mask_path =
        JoinPaths(options_.mask_path,
                  GetRelativePath(options_.image_path, image_path) + ".png");
    if (ExistsFile(mask_path) && !mask->Read(mask_path, false)) {
      // NOTE: Maybe introduce a separate error type MASK_ERROR?
      return Status::BITMAP_ERROR;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Check for well-formed data.
  //////////////////////////////////////////////////////////////////////////////

  if (exists_image) {
    const Camera current_camera = database_->ReadCamera(image->CameraId());

    if (options_.single_camera && prev_camera_.CameraId() != kInvalidCameraId &&
        (current_camera.Width() != prev_camera_.Width() ||
         current_camera.Height() != prev_camera_.Height())) {
      return Status::CAMERA_SINGLE_DIM_ERROR;
    }

    if (static_cast<size_t>(bitmap->Width()) != current_camera.Width() ||
        static_cast<size_t>(bitmap->Height()) != current_camera.Height()) {
      return Status::CAMERA_EXIST_DIM_ERROR;
    }

    prev_camera_ = current_camera;

  } else {
    //////////////////////////////////////////////////////////////////////////////
    // Check image dimensions.
    //////////////////////////////////////////////////////////////////////////////

    if (prev_camera_.CameraId() != kInvalidCameraId &&
        ((options_.single_camera && !options_.single_camera_per_folder) ||
         (options_.single_camera_per_folder &&
          image_folder == prev_image_folder_)) &&
        (prev_camera_.Width() != static_cast<size_t>(bitmap->Width()) ||
         prev_camera_.Height() != static_cast<size_t>(bitmap->Height()))) {
      return Status::CAMERA_SINGLE_DIM_ERROR;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Read camera model and check for consistency if it exists
    //////////////////////////////////////////////////////////////////////////////
    std::string camera_model;
    const bool valid_camera_model = bitmap->ExifCameraModel(&camera_model);
    if (camera_model_to_id_.count(camera_model) > 0) {
      const Camera& cam =
          database_->ReadCamera(camera_model_to_id_.at(camera_model));
      if (cam.Width() != static_cast<size_t>(bitmap->Width()) ||
          cam.Height() != static_cast<size_t>(bitmap->Height())) {
        return Status::CAMERA_EXIST_DIM_ERROR;
      }
      prev_camera_ = cam;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Extract camera model and focal length
    //////////////////////////////////////////////////////////////////////////////

    if (prev_camera_.CameraId() == kInvalidCameraId ||
        options_.single_camera_per_image ||
        (!options_.single_camera && !options_.single_camera_per_folder &&
         static_cast<camera_t>(options_.existing_camera_id) ==
             kInvalidCameraId &&
         camera_model_to_id_.count(camera_model) == 0) ||
        (options_.single_camera_per_folder &&
         image_folders_.count(image_folder) == 0)) {
      if (options_.camera_params.empty()) {
        // Extract focal length.
        double focal_length = 0.0;
        if (bitmap->ExifFocalLength(&focal_length)) {
          prev_camera_.SetPriorFocalLength(true);
        } else {
          focal_length = options_.default_focal_length_factor *
                         std::max(bitmap->Width(), bitmap->Height());
          prev_camera_.SetPriorFocalLength(false);
        }

        prev_camera_.InitializeWithId(prev_camera_.ModelId(), focal_length,
                                      bitmap->Width(), bitmap->Height());
      }

      prev_camera_.SetWidth(static_cast<size_t>(bitmap->Width()));
      prev_camera_.SetHeight(static_cast<size_t>(bitmap->Height()));

      if (!prev_camera_.VerifyParams()) {
        return Status::CAMERA_PARAM_ERROR;
      }
      if (options_.camera_non_svp_model != "NONE") {
        if (!prev_camera_.VerifyNonSvpParams()) {
          return Status::CAMERA_PARAM_ERROR;
        }
      }

      prev_camera_.SetCameraId(database_->WriteCamera(prev_camera_));
      if (valid_camera_model) {
        camera_model_to_id_[camera_model] = prev_camera_.CameraId();
      }
    }

    image->SetCameraId(prev_camera_.CameraId());

    //////////////////////////////////////////////////////////////////////////////
    // Extract pose prior data.
    //////////////////////////////////////////////////////////////////////////////

    if (options_.read_pose_prior_from_csv && have_csv_file) {
      boost::filesystem::ifstream fileStream(csv_path);
      std::string csv_header, csv;
      getline(fileStream, csv_header);

      auto csv_header_values = CSVToVector<std::string>(csv_header);

      auto iter_long =
          std::distance(csv_header_values.begin(),
                        std::find(csv_header_values.begin(),
                                  csv_header_values.end(), "longitude"));
      auto iter_lat =
          std::distance(csv_header_values.begin(),
                        std::find(csv_header_values.begin(),
                                  csv_header_values.end(), "latitude"));
      auto iter_depth =
          std::distance(csv_header_values.begin(),
                        std::find(csv_header_values.begin(),
                                  csv_header_values.end(), "depth"));
      auto iter_yaw = std::distance(
          csv_header_values.begin(),
          std::find(csv_header_values.begin(), csv_header_values.end(), "yaw"));
      auto iter_pitch =
          std::distance(csv_header_values.begin(),
                        std::find(csv_header_values.begin(),
                                  csv_header_values.end(), "pitch"));
      auto iter_roll =
          std::distance(csv_header_values.begin(),
                        std::find(csv_header_values.begin(),
                                  csv_header_values.end(), "roll"));

      // TODO check if existent

      getline(fileStream, csv);

      auto csv_values = CSVToVector<std::string>(csv);

      auto latitude = atof(csv_values.at(iter_lat).c_str());
      auto longitude = atof(csv_values.at(iter_long).c_str());
      auto depth = atof(csv_values.at(iter_depth).c_str());
      auto yaw = atof(csv_values.at(iter_yaw).c_str());
      auto pitch = atof(csv_values.at(iter_pitch).c_str());
      auto roll = atof(csv_values.at(iter_roll).c_str());

      Eigen::Vector3d tvec_prior_to_world;

#ifdef GDAL_ENABLED

      // Converting Lat,Long to UTM coordinate system.
      // image->TvecPrior(0) = UTM_northing (utm.y)
      // image->TvecPrior(1) = UTM_easting (utm.x)
      // image->TvecPrior(2) = depth or (AVERAGE_WATER_DEPTH - altitude)
      GeoConversion geo_conversion;  // todo set zone
      int utm_zone = geo_conversion.ComputeUTMZone(longitude);
      geo_conversion.SetUTMZone(utm_zone);
      auto position_utm = geo_conversion.WGS84ToUTM(longitude, latitude, depth);

      tvec_prior_to_world[0] = position_utm.y();
      tvec_prior_to_world[1] = position_utm.x();
      tvec_prior_to_world[2] = position_utm.z();
#else
      tvec_prior_to_world[0] = latitude;
      tvec_prior_to_world[1] = longitude;
      tvec_prior_to_world[2] = depth;
#endif

      if (options_.normalize_tvec_priors) {
        if (global_tvec_prior_offset_[0] == -999999.9 &&
            global_tvec_prior_offset_[1] == -999999.9 &&
            global_tvec_prior_offset_[2] == -999999.9) {
          global_tvec_prior_offset_ = tvec_prior_to_world;
        }
        tvec_prior_to_world -= global_tvec_prior_offset_;
      }

      // Read yaw, pitch, roll and compute rotation out of it:
      // yaw: rotation around z-axis
      // pitch: rotation around y-axis
      // roll: rotation around x-axis
      // Rotation matrix is computed as: R = Rz * Ry * Rx ("ZYX" order)
      // Note: This rotation matrix rotates a point in the prior coordinate
      // system to the world coordinate system

      Eigen::Matrix3d R_prior_to_world =
          EulerAnglesToRotationMatrix(roll, pitch, yaw);

      const Eigen::Vector4d qvec_prior_to_world =
          RotationMatrixToQuaternion(R_prior_to_world);
      Eigen::Vector4d qvec_world_to_prior;
      Eigen::Vector3d tvec_world_to_prior;

      InvertPose(qvec_prior_to_world, tvec_prior_to_world, &qvec_world_to_prior,
                 &tvec_world_to_prior);
      image->SetQvecPrior(qvec_world_to_prior);
      image->SetTvecPrior(tvec_world_to_prior);

    } else {
      if (!bitmap->ExifLatitude(&image->TvecPrior(0)) ||
          !bitmap->ExifLongitude(&image->TvecPrior(1)) ||
          !bitmap->ExifAltitude(&image->TvecPrior(2))) {
        image->TvecPrior().setConstant(
            std::numeric_limits<double>::quiet_NaN());
      }
    }
  }

  *camera = prev_camera_;

  image_folders_.insert(image_folder);
  prev_image_folder_ = image_folder;

  return Status::SUCCESS;
}

size_t ImageReader::NextIndex() const { return image_index_; }

size_t ImageReader::NumImages() const { return options_.image_list.size(); }

}  // namespace colmap
