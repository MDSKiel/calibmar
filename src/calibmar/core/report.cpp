#include "report.h"

#include "calibmar/core/calibration_targets.h"

#include "colmap/util/misc.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>

namespace {
  std::vector<std::string> Split(const std::string& input, const std::string& regex) {
    // passing -1 as the submatch index parameter performs splitting
    std::regex re(regex);
    std::sregex_token_iterator first{input.begin(), input.end(), re, -1}, last;
    return {first, last};
  }

  std::string OriginToString(calibmar::ArucoGridOrigin origin) {
    switch (origin) {
      case calibmar::ArucoGridOrigin::TopLeft:
        return "tl";
      case calibmar::ArucoGridOrigin::TopRight:
        return "tr";
      case calibmar::ArucoGridOrigin::BottomLeft:
        return "bl";
      case calibmar::ArucoGridOrigin::BottomRight:
        return "br";
      default:
        return "unkown";
    }
  }

  std::string DirectionToString(calibmar::ArucoGridDirection direction) {
    switch (direction) {
      case calibmar::ArucoGridDirection::Horizontal:
        return "hor";
      case calibmar::ArucoGridDirection::Vertical:
        return "ver";
      default:
        return "unkown";
    }
  }

  calibmar::ArucoGridOrigin StringToOrigin(const std::string& origin) {
    calibmar::ArucoGridOrigin res;
    if (origin == "tl") {
      res = calibmar::ArucoGridOrigin::TopLeft;
    }
    else if (origin == "tr") {
      res = calibmar::ArucoGridOrigin::TopRight;
    }
    else if (origin == "bl") {
      res = calibmar::ArucoGridOrigin::BottomLeft;
    }
    else if (origin == "br") {
      res = calibmar::ArucoGridOrigin::BottomRight;
    }
    return res;
  }

  calibmar::ArucoGridDirection StringToDirection(const std::string& direction) {
    calibmar::ArucoGridDirection dir;
    if (direction == "hor") {
      dir = calibmar::ArucoGridDirection::Horizontal;
    }
    else if (direction == "ver") {
      dir = calibmar::ArucoGridDirection::Vertical;
    }
    return dir;
  }
}

namespace calibmar {
  namespace report {
    void WriteCalibrationReport(const std::string& path, const Calibration& calibration) {
      std::ofstream out_file(path);
      GenerateResultString(out_file, calibration);
      out_file.close();
    }

    void WriteCalibrationYaml(const std::string& path, const Calibration& calibration) {
      std::ofstream out_file(path);
      out_file.precision(10);
      GenerateCalibrationYaml(out_file, calibration);
      out_file.close();
    }

    void GenerateCalibrationYaml(std::ostream& stream, const Calibration& calibration) {
      const colmap::Camera& camera = calibration.Camera();
      // model
      stream << "model: " << camera.ModelName() << std::endl;
      stream << "# " << camera.ParamsInfo() << std::endl;
      // parameters
      stream << "parameters: [";
      if (camera.params.size() > 0) {
        const std::vector<double>& params = camera.params;
        std::copy(params.begin(), params.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(params.end() - 1);
      }

      stream << "]" << std::endl;
      // std dev parameters
      const std::vector<double>& intrinsics_std_dev = calibration.IntrinsicsStdDeviations();
      if (intrinsics_std_dev.size() > 0) {
        stream << "# params_est_std_dev: ";
        std::copy(intrinsics_std_dev.begin(), intrinsics_std_dev.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(intrinsics_std_dev.end() - 1) << std::endl;
      }
      // non_svp
      std::string non_svp_model_name = camera.IsCameraRefractive() ? camera.RefracModelName() : "NONE";
      stream << "non_svp_model: " << non_svp_model_name << std::endl;
      if (camera.IsCameraRefractive()) {
        // one of the info strings contains a new line...
        std::string params_info = std::regex_replace(camera.RefracParamsInfo(), std::regex("\\n"), " ");
        stream << "# " << params_info << std::endl;
      }
      stream << "non_svp_parameters: [";
      if (camera.IsCameraRefractive()) {
        const std::vector<double>& non_svp_params = camera.refrac_params;
        std::copy(non_svp_params.begin(), non_svp_params.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(non_svp_params.end() - 1);
      }
      stream << "]" << std::endl;
      // std dev parameters non svp
      const std::vector<double>& housing_std_dev = calibration.HousingParamsStdDeviations();
      if (housing_std_dev.size() > 0) {
        stream << "# non_svp_est_std_dev: ";
        std::copy(housing_std_dev.begin(), housing_std_dev.end() - 1, std::ostream_iterator<double>(stream, ", "));
        stream << *(housing_std_dev.end() - 1) << std::endl;
      }
      // width & height
      stream << "width: " << camera.width << std::endl;
      stream << "height: " << camera.height << std::endl;
      // overall RMS
      stream << "# overall_rms: " << calibration.CalibrationRms() << std::endl;
      // target
      stream << "# target: " << calibration.GetCalibrationTargetInfo();

      // stereo
      if (calibration.CameraToWorldStereo().has_value()) {
        Eigen::IOFormat yaml_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
        const colmap::Rigid3d& pose = calibration.CameraToWorldStereo().value();
        stream << "\n\n# stereo pose (camera to world, camera 3D point X_C can be transformed to 3D world point X_W by X_W = "
                  "cam_to_world_rotation * X_C + cam_to_world_translation)";
        stream << "\ncam_to_world_rotation_rowmajor: " << pose.rotation.normalized().toRotationMatrix().format(yaml_format);
        stream << "\ncam_to_world_translation: " << pose.translation.format(yaml_format);
        stream << "\n# (distance to origin: " << pose.translation.norm() << ")";
      }
    }

    std::string GenerateResultString(const Calibration& calibration) {
      std::stringstream result_stream;
      GenerateResultString(result_stream, calibration);
      return result_stream.str();
    }

    void GenerateResultString(std::ostream& stream, const Calibration& calibration) {
      int first_column_width = 22;
      int field_width = 13;
      const colmap::Camera& camera = calibration.Camera();
      // camera model
      std::string header = camera.IsCameraRefractive() ? "Camera & Housing Model:" : "Camera Model:";
      stream << header << std::endl << camera.ModelName();
      if (camera.IsCameraRefractive()) {
        stream << " " << camera.RefracModelName();
      }
      stream << std::endl << std::endl;
      // width & height
      stream << "Width & Height:" << std::endl;
      stream << camera.width << " " << camera.height;
      stream << std::endl << std::endl;
      // camera matrix
      stream << "Camera Matrix:" << std::endl << camera.CalibrationMatrix() << std::endl << std::endl;
      // parameter lables
      std::vector<std::string> param_names = Split(camera.ParamsInfo(), ", ");
      stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Parameters:";
      for (auto& value : param_names) {
        stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      }
      stream << std::endl;
      // parameter values
      const std::vector<double>& params = camera.params;
      stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
      for (auto& value : params) {
        stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
      }
      stream << std::endl;
      // parameter std dev
      const std::vector<double>& intrinsics_std_dev = calibration.IntrinsicsStdDeviations();
      if (intrinsics_std_dev.size() > 0) {
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
        for (auto& value : intrinsics_std_dev) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
      }
      if (camera.IsCameraRefractive()) {
        stream << std::endl;
        // housing labels
        std::vector<std::string> housing_param_names = Split(camera.RefracParamsInfo(), ", ");
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Housing Parameters:";
        for (auto& value : housing_param_names) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
        // housing values
        const std::vector<double>& housing_params = camera.refrac_params;
        stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Values:";
        for (auto& value : housing_params) {
          stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
        }
        stream << std::endl;
        // housing std dev
        const std::vector<double>& housing_std_dev = calibration.HousingParamsStdDeviations();
        if (housing_std_dev.size() > 0) {
          stream << std::left << std::setw(first_column_width) << std::setfill(' ') << "Est. Std. Deviations:";
          for (auto& value : housing_std_dev) {
            stream << std::left << std::setw(field_width) << std::setfill(' ') << value;
          }
          stream << std::endl;
        }
      }
      // optional stereo pose
      if (calibration.CameraToWorldStereo().has_value()) {
        const colmap::Rigid3d& cam_to_world = calibration.CameraToWorldStereo().value();
        stream << std::endl << "Stereo Pose (camera to world R|t):" << std::endl;
        stream << cam_to_world.ToMatrix() << std::endl;
        stream << "(Distance to origin: " << cam_to_world.translation.norm() << ")" << std::endl;
      }
      // overall rms
      stream << std::endl << "Overall RMS: " << calibration.CalibrationRms();
    }

    std::string GenerateCalibrationTargetInfo(const ChessboardFeatureExtractor::Options& options) {
      return "chessboard, " + std::to_string(options.chessboard_columns) + ", " + std::to_string(options.chessboard_rows) + ", " +
             std::to_string(options.square_size);
    }

    std::string GenerateCalibrationTargetInfo(const ArucoBoardFeatureExtractor::Options& options) {
      return "aruco grid board, " + calibration_targets::NameFromArucoType(options.aruco_type) + ", " +
             std::to_string(options.marker_cols) + ", " + std::to_string(options.marker_rows) + ", " +
             std::to_string(options.marker_size) + ", " + std::to_string(options.marker_spacing) + ", " +
             std::to_string(options.border_bits) + ", " + OriginToString(options.grid_origin) + ", " +
             DirectionToString(options.grid_direction);
    }

    std::string GenerateCalibrationTargetInfo(
        const std::variant<ArucoSiftFeatureExtractor::Options, SiftFeatureExtractor::Options>& options) {
      const ArucoSiftFeatureExtractor::Options* aruco_options = std::get_if<ArucoSiftFeatureExtractor::Options>(&options);
      if (aruco_options) {
        return "3D target, aruco: " + calibration_targets::NameFromArucoType(aruco_options->aruco_type) + ", " +
               std::to_string(aruco_options->masking_scale_factor);
      }
      else {
        return "3D target";
      }
    }
  }

  namespace {
    bool StartsWith(const std::string& string, const std::string& target) {
      return string.find(target) == 0;
    }

    inline void TrimEnd(std::string& s) {
      s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
    }

    inline void TrimStart(std::string& s) {
      s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
    }

    inline void ToLower(std::string& s) {
      std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    }

    bool TryParse(double* d, const std::string& string, size_t* chars_used) {
      try {
        *d = std::stod(string, chars_used);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    bool TryParse(int* i, const std::string& string, size_t* chars_used) {
      try {
        *i = std::stoi(string, chars_used);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    template <typename T>
    bool TryParse(T* i, const std::string& string) {
      static_assert(sizeof(T) != sizeof(T), "TryParse must be specialized for this type!");
      return false;
    }

    template <>
    bool TryParse<int>(int* i, const std::string& string) {
      try {
        *i = std::stoi(string);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    template <>
    bool TryParse<double>(double* d, const std::string& string) {
      try {
        *d = std::stod(string);
        return true;
      }
      catch (std::exception& e) {
        return false;
      }
    }

    template <>
    bool TryParse<calibmar::ArucoMarkerTypes>(calibmar::ArucoMarkerTypes* type, const std::string& string) {
      for (const auto& name_type : calibration_targets::ArucoTypes()) {
        std::string aruco_name = name_type.first;
        std::string value_copy = string;
        ToLower(aruco_name);
        ToLower(value_copy);
        if (StartsWith(value_copy, aruco_name)) {
          *type = name_type.second;
          return true;
        }
      }
      return false;
    }

    template <>
    bool TryParse<calibmar::ArucoGridOrigin>(calibmar::ArucoGridOrigin* origin, const std::string& string) {
      *origin = StringToOrigin(string);
      return true;
    }

    template <>
    bool TryParse<calibmar::ArucoGridDirection>(calibmar::ArucoGridDirection* direction, const std::string& string) {
      *direction = StringToDirection(string);
      return true;
    }

    template <typename T>
    bool Parse(const std::vector<std::string>& values, size_t index, T* parameter) {
      return index < values.size() && TryParse(parameter, values[index]);
    }
  }

  ImportedParameters ImportedParameters::ImportFromYaml(const std::string& path) {
    ImportedParameters parameters;

    std::filesystem::path file_path(path);
    if (!std::filesystem::exists(file_path) || !file_path.has_filename()) {
      return parameters;
    }

    std::ifstream input(path);

    parameters = ImportFromYaml(input);
    parameters.directory = file_path.parent_path().string();

    return parameters;
  }

  ImportedParameters ImportedParameters::ImportFromYaml(std::istream& stream) {
    ImportedParameters parameters;
    std::string line;
    while (std::getline(stream, line)) {
      TrimEnd(line);
      if (StartsWith(line, "model:")) {
        line.erase(0, sizeof("model: ") - 1);
        for (auto& [type, model] : CameraModel::CameraModels()) {
          if (model.model_name == line) {
            parameters.camera_model = type;
            break;
          }
        }
      }
      else if (StartsWith(line, "parameters: [")) {
        line.erase(0, sizeof("parameters: [") - 1);
        double p;
        size_t chars_used;
        while (TryParse(&p, line, &chars_used)) {
          parameters.camera_parameters.push_back(p);
          // + the comma or closing bracket
          line.erase(0, chars_used + 1);
        }
      }
      else if (StartsWith(line, "non_svp_model:")) {
        line.erase(0, sizeof("non_svp_model: ") - 1);
        for (auto& [type, model] : HousingInterface::HousingInterfaces()) {
          if (model.model_name == line) {
            parameters.housing_model = type;
            break;
          }
        }
      }
      else if (StartsWith(line, "non_svp_parameters: [")) {
        line.erase(0, sizeof("non_svp_parameters: [") - 1);
        double p;
        size_t chars_used;
        while (TryParse(&p, line, &chars_used)) {
          parameters.housing_parameters.push_back(p);
          // + the comma or closing bracket
          line.erase(0, chars_used + 1);
        }
      }
      else if (StartsWith(line, "# target: chessboard,")) {
        parameters.calibration_target = CalibrationTargetType::Chessboard;
        line.erase(0, sizeof("# target: chessboard,") - 1);

        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.columns);
        Parse(values, 1, &parameters.rows);
        Parse(values, 2, &parameters.square_size);
      }
      else if (StartsWith(line, "# target: 3D target, aruco:")) {
        parameters.calibration_target = CalibrationTargetType::Target3DAruco;
        line.erase(0, sizeof("# target: 3D target, aruco:") - 1);

        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.aruco_type);
        Parse(values, 1, &parameters.aruco_scale_factor);
      }
      else if (StartsWith(line, "# target: 3D target")) {
        parameters.calibration_target = CalibrationTargetType::Target3D;
      }
      else if (StartsWith(line, "# target: aruco grid board,")) {
        parameters.calibration_target = CalibrationTargetType::ArucoGridBoard;
        line.erase(0, sizeof("# target: aruco grid board,") - 1);

        std::vector<std::string> values = colmap::CSVToVector<std::string>(line);
        Parse(values, 0, &parameters.aruco_type);
        Parse(values, 1, &parameters.columns);
        Parse(values, 2, &parameters.rows);
        Parse(values, 3, &parameters.square_size);
        Parse(values, 4, &parameters.spacing);
        Parse(values, 5, &parameters.border_bits);
        Parse(values, 6, &parameters.grid_origin);
        Parse(values, 7, &parameters.grid_direction);
      }
    }

    return parameters;
  }
}
